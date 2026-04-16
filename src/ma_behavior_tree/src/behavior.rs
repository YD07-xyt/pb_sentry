mod alert;
mod attack;
mod nav;
mod retreat;

pub(crate) mod sentry_state;
use std::{sync::Arc, time::Duration};

use async_ctrlc::CtrlC;
use async_shutdown::ShutdownManager;
use futures_util::FutureExt as _;
use ros2_client::{Node, Publisher, Subscription, action::ActionClient};
use sentry_state::{GameStatus, SentryStates, State};
use smol::{Timer, future::FutureExt as _};

use crate::{
    BTState,
    dedup::Dedup,
    msgs::{
        nav2_msgs::action::NavigateToPose,
        serial_msgs::{SerialReceiveData, SerialReceiveHPData},
    },
};

type Shutdown = ShutdownManager<isize>;

pub struct SentryCtx {
    pub(crate) node: Node,
    pub(crate) nav_clinet: ActionClient<NavigateToPose::Action>,
    pub(crate) states: Arc<SentryStates>,
    pub(crate) state_publisher: Publisher<BTState>,
}

impl SentryCtx {
    pub fn new(
        node: Node,
        nav_clinet: ActionClient<NavigateToPose::Action>,
        serial_subscription: Subscription<SerialReceiveData>,
        serial_hp_subscription: Subscription<SerialReceiveHPData>,
        state_publisher: Publisher<BTState>,
    ) -> Result<Self, anyhow::Error> {
        let states = SentryStates::default();
        let states = Arc::new(states);

        let sender_states = Arc::clone(&states);
        let sender_hp_states = Arc::clone(&states);

        smol::spawn(async move {
            use futures_util::{StreamExt, TryStreamExt};
            serial_subscription
                .async_stream()
                .map_ok(|(msg, _)| {
                    let SerialReceiveData { game_progress, .. } = msg;
                    game_progress
                })
                .inspect_err(|e| log::error!("Error: {e}"))
                .filter_map(|x| async { x.ok() })
                .for_each(async |status| {
                    sender_states.game_status.store(status);
                    // log::trace!("broadcast: game_status: {game_status:?}, hp: {hp:?}, outpost_hp: {outpost_hp:?}")
                })
                .await
        })
        .detach();

        smol::spawn(async move {
            use futures_util::{StreamExt, TryStreamExt};
            serial_hp_subscription
                .async_stream()
                .map_ok(|(msg, _)| {
                    let SerialReceiveHPData { current_hp, .. } = msg;
                    current_hp
                })
                .inspect_err(|e| log::error!("Error: {e}"))
                .filter_map(|x| async { x.ok() })
                .for_each(async |hp| {
                    sender_hp_states.hp.store(hp);
                })
                .await
        })
        .detach();

        Ok(Self {
            node,
            nav_clinet,
            states,
            state_publisher,
        })
    }

    pub async fn sentry_task(&mut self, ctrlc: &mut CtrlC) -> Result<(), anyhow::Error> {
        let status = &self.states;

        if let None = status
            .game_status
            .wait_until(|game_status| game_status == GameStatus::InGame)
            .inspect(|_| log::info!("Game is started"))
            .map(Some)
            .or(async {
                (&mut *ctrlc).await;
                None
            })
            .await
        {
            return Ok(());
        }

        let shutdown = async_shutdown::ShutdownManager::new();

        nav::patrol(self, shutdown.clone())
            .or(async {
                ctrlc.await;
                let _ = shutdown.trigger_shutdown(0);
                shutdown.wait_shutdown_complete().await;
                Ok(())
            })
            .or(async {
                status
                    .game_status
                    .wait_until(|game_status| game_status != GameStatus::InGame)
                    .map(|_| log::info!("Game is ended"))
                    .or(status
                        .hp
                        .wait_until(|hp| hp == 0)
                        .map(|_| log::info!("HP is 0")))
                    .await;
                let _ = shutdown.trigger_shutdown(0);
                shutdown.wait_shutdown_complete().await;
                Err(anyhow::anyhow!("Game is ended"))
            })
            .or(async {
                Timer::after(Duration::from_secs(30)).await;
                status.hp.wait_until(|hp| hp < 50).await;
                let _ = shutdown.trigger_shutdown(1);
                shutdown.wait_shutdown_complete().await;
                retreat::retreat(self, shutdown.clone()).await?;
                Err(anyhow::anyhow!("HP is low"))
            })
            // .or(async {
            //     let mut last_hp = status.hp.load();
            //     status
            //         .hp
            //         .wait_until(|hp| {
            //             let should_alert = hp < last_hp;
            //             last_hp = hp;
            //             should_alert
            //         })
            //         .await;
            //     let _ = shutdown.trigger_shutdown(1);
            //     alert::alert(status, &self.state_publisher, shutdown.clone()).await
            // })
            // .or(async {
            //     status.outpost_hp.wait_until(|outpost_hp| outpost_hp > 0).await;
            //     attack::attack_outpost(&self, terminate_signal.clone()).await
            // })
            .await
    }
}
