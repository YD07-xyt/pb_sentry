use crate::msgs::geometry_msgs::Point;

use super::{
    SentryCtx, Shutdown, nav,
    sentry_state::{HpState, State},
};

pub async fn retreat(ctx: &SentryCtx, shutdown: Shutdown) -> Result<(), anyhow::Error> {
    nav::nav_to_point(
        ctx,
        Point {
            x: 0.0,
            y: -2.0,
            z: 0.0,
        },
        shutdown,
    )
    .await?;
    ctx.states.hp.wait_until(|hp| hp >= HpState::FULL).await;
    Ok(())
}
