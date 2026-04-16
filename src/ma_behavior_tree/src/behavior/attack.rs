use crate::msgs::geometry_msgs::Point;

use super::{SentryCtx, Shutdown, nav};

#[expect(unused)]
pub async fn attack_outpost(ctx: &SentryCtx, shutdown: Shutdown) -> Result<(), anyhow::Error> {
    nav::nav_to_point(
        ctx,
        Point {
            x: 3.0,
            y: 2.0,
            z: 0.0,
        },
        shutdown,
    )
    .await
}
