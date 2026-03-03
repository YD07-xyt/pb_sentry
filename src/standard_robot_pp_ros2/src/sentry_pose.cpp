#include"standard_robot_pp_ros2/sentry_pose.hpp"
#include <cstdio>

namespace rmDecision {
    PosTransform::PosTransform(int sentry_hp,bool is_in_game){
        sentry_pose =SentryPose::Move;
        if(sentry_hp>400||sentry_hp<0){
            printf("[warn],传入的sentry_hp>400||<0");
            return;
        }
        sentry_hp_=sentry_hp;
        is_in_game_=is_in_game;
    }
    SentryPose PosTransform::pos_to_new(){
        if(!is_in_game_){
            printf("不在游戏中，哨兵姿态不切换");
            return SentryPose::Move;
        }
        PosToDefend(sentry_hp_);
        PosToMove(sentry_hp_);
        return sentry_pose;
    }

    void PosTransform::PosToAttack(int sentry_hp){
        if(sentry_pose == SentryPose::Attack){
            return;
        }
    }

    void PosTransform::PosToDefend(int sentry_hp){
        if(sentry_pose == SentryPose::Defend){
            return;
        }
        if(sentry_hp<200){
            sentry_pose=SentryPose::Defend;
        }
    }

    void PosTransform::PosToMove(int sentry_hp){
        if(sentry_pose == SentryPose::Move){
            return;
        }
        if(sentry_hp>=200){
            sentry_pose=SentryPose::Move;
        }
    }
}