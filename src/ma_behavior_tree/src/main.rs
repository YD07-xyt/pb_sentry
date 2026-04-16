mod behavior;
mod builder;
mod dedup;
mod msgs;
mod sentry_action;

use async_ctrlc::CtrlC;
use msgs::nav2_msgs::action::NavigateToPose;
use ros2_client::action::ActionClientQosPolicies;
use serde::Serialize;

#[derive(Debug, Serialize)]
struct BTState {
    pub spin: bool,
}

fn main() -> Result<(), anyhow::Error> {
    use builder::*;
    env_logger::init();
    let mut node = Ros2NodeBuilder::new()
        .name("/ma", "behavior_tree")
        .build()?;

    let nav_client = node
        .action_client_builder::<NavigateToPose::Action>()
        .action_base_name("navigate_to_pose")
        .type_name("nav2_msgs", "NavigateToPose")
        .qos(ActionClientQosPolicies {
            goal_service: Default::default(),
            result_service: Default::default(),
            cancel_service: Default::default(),
            feedback_subscription: Default::default(),
            status_subscription: Default::default(),
        })
        .build()?;
    //pb_rm_interfaces::msg::GameStatus
    //referee/game_status
    let serial_subscription = node
        .pub_sub_builder::<msgs::serial_msgs::SerialReceiveData>()
        .topic_name("/referee", "game_status")
        .type_name("pb_rm_interfaces", "GameStatus")
        .build_subscription()?;

    let serial_hp_subscription = node
        .pub_sub_builder::<msgs::serial_msgs::SerialReceiveHPData>()
        .topic_name("/referee", "robot_status")
        .type_name("pb_rm_interfaces", "RobotStatus")
        .build_subscription()?;

    let bt_state_publisher = node
        .pub_sub_builder::<BTState>()
        .topic_base_name("cmd_bt_state")
        .type_name("bt_msgs", "BTState")
        .build_publisher()?;

    let mut ctrlc = CtrlC::new().expect("cannot create Ctrl+C handler?");

    smol::spawn(node.spinner()?.spin()).detach();

    smol::block_on(async {
        let mut ctx = behavior::SentryCtx::new(
            node,
            nav_client,
            serial_subscription,
            serial_hp_subscription,
            bt_state_publisher,
        )?;

        while let Err(e) = ctx.sentry_task(&mut ctrlc).await {
            log::warn!("sentry task failed: {e}, restart the bt");
        }

        Ok(())
    })
}
