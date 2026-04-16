use std::time::Duration;

use anyhow::anyhow;
use smol::{Timer, future::FutureExt as _};

use crate::{
    msgs::geometry_msgs::Point,
    sentry_action::nav_to_pose::{self as nav_action},
};

use super::{SentryCtx, Shutdown};

pub async fn patrol(ctx: &SentryCtx, shutdown: Shutdown) -> Result<(), anyhow::Error> {
    loop {
        nav_to_point(
            ctx,
            Point {
                x: 0.719,
                y: -1.46,
                z: 0.0,
            },
            shutdown.clone(),
        )
        .await?;

        Timer::after(Duration::from_secs(1)).await;
    }
}

/// when shutdown signal is triggered, cancel the goal and return an error
pub async fn nav_to_point(
    ctx: &SentryCtx,
    point: Point,
    shutdown: Shutdown,
) -> Result<(), anyhow::Error> {
    log::info!("Start nav to point: {point:?}");
    let client = &ctx.nav_clinet;

    let (goal_id, stamp) = loop {
        match nav_action::start_nav(client, point.clone(), ctx.node.time_now()).await {
            Ok(result) => break result,
            Err(e) => {
                log::warn!("Fail to send goal: {e}");
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    };
    async {
        if let Err(e) = nav_action::request_result(client, goal_id).await {
            log::error!("Fail to request result: {e}")
        }
        Ok(())
    }
    .or(shutdown.wrap_delay_shutdown(async {
        let reason = shutdown.wait_shutdown_triggered().await;
        log::info!("Canceling goal: {goal_id:?}");
        if let Err(e) = nav_action::cancel_goal(client, goal_id, stamp).await {
            log::error!("Fail to cancel goal: {e}")
        }
        Ok(())
    }).unwrap())
    .await
}
