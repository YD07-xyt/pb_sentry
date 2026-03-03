#pragma once 

#ifndef SENTRY_POSE_HPP
#define SENTRY_POSE_HPP

namespace rmDecision{
    enum class SentryPose{
        Attack=2,
        Defend=1,
        Move=0,
    };

    class PosTransform {
        public:
            PosTransform(int sentry_hp,bool is_in_game);
            SentryPose pos_to_new();
        private:
            void PosToMove(int sentry_hp);
            void PosToAttack(int sentry_hp);
            void PosToDefend(int sentry_hp);
        private:
            SentryPose sentry_pose;
            int sentry_hp_;
            bool is_in_game_;
    };
}
#endif