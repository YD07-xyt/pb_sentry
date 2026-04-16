pub mod error;
use std::time::Duration;

pub use error::{CancelGoalError, RequestResultError, SendGoalError};
use futures_util::{FutureExt, StreamExt, TryFutureExt, TryStreamExt};
use ros2_client::{
    ROSTime,
    action::{ActionClient, GoalInfo, GoalStatusEnum},
    action_msgs::CancelGoalResponseEnum,
    builtin_interfaces::Time,
    unique_identifier_msgs::UUID,
};
use smol::future::FutureExt as SmolFutureExt;

use crate::msgs::{
    geometry_msgs::{Point, Pose, PoseStamped},
    nav2_msgs::action::NavigateToPose,
    std_msgs::Header,
};
pub type Client = ActionClient<NavigateToPose::Action>;

pub async fn start_nav(
    client: &Client,
    point: Point,
    time: ROSTime,
) -> Result<(UUID, Time), SendGoalError> {
    let send_goal_with_time_out_result = client
        .async_send_goal(NavigateToPose::Goal {
            pose: PoseStamped {
                header: Header {
                    stamp: time.into(),
                    frame_id: "map".to_string(),
                },
                pose: Pose {
                    position: point,
                    ..Default::default()
                },
            },
            behavior_tree: String::new(),
        })
        .map_err(From::from)
        .and_then(|(uuid, response)| async move {
            if response.accepted {
                Ok((uuid, response.stamp))
            } else {
                Err(SendGoalError::Rejected)
            }
        })
        .map(Result::Ok)
        .or(async {
            let instance = smol::Timer::after(Duration::from_secs(3)).await;
            Err(SendGoalError::Timeout(instance))
        })
        .await;

    let (uuid, stamp) = send_goal_with_time_out_result??;

    log::info!("Goal was accepted by the server, uuid: {uuid:?}, timestamp: {stamp:?}",);
    Ok((uuid, stamp))
}

#[expect(unused)]
pub async fn log_feedback(client: &Client, goal_id: UUID) {
    client
        .feedback_stream(goal_id)
        .inspect_err(|e| log::error!("Error when get feedback: {e:?}"))
        .filter_map(|result| async { result.ok() })
        .for_each(|feedback| async move {
            log::trace!(
                "Sentry is moving, current position: {:?}",
                feedback.current_pose
            );
        })
        .await;
}

pub async fn request_result(client: &Client, goal_id: UUID) -> Result<(), RequestResultError> {
    let (status, _result) = client.async_request_result(goal_id).await?;

    log::info!("Goal status: {:?}", status);

    matches!(status, GoalStatusEnum::Succeeded)
        .then_some(())
        .ok_or(RequestResultError::GoalNotSuccessful)
}

pub async fn cancel_goal(
    client: &Client,
    goal_id: UUID,
    timestamp: Time,
) -> Result<Vec<GoalInfo>, CancelGoalError> {
    let response = client.async_cancel_goal(goal_id, timestamp).await?;
    match response.return_code {
        CancelGoalResponseEnum::None => Ok(response.goals_canceling),
        CancelGoalResponseEnum::Rejected => Err(CancelGoalError::Rejected),
        CancelGoalResponseEnum::UnknownGoal => Err(CancelGoalError::UnknownGoal),
        CancelGoalResponseEnum::GoalTerminated => Err(CancelGoalError::GoalTerminated),
    }
}
