#include <icub/model/iCub_common.hpp>

namespace icub {
    namespace model {
        std::vector<std::pair<std::string, std::string>> packages()
        {
            return {std::make_pair("iCub", std::string(RESPATH))};
        }

        std::vector<std::string> bodies()
        {
            return {
                "base_link",
                "root_link",
                "l_hip_1",
                "l_hip_2",
                "l_hip_3",
                "l_upper_leg",
                "l_lower_leg",
                "l_ankle_1",
                "l_ankle_2",
                "l_foot",
                "l_foot_dh_frame",
                "l_sole",
                "l_upper_leg_back_contact",
                "l_upper_leg_dh_frame",
                "r_hip_1",
                "r_hip_2",
                "r_hip_3",
                "r_upper_leg",
                "r_lower_leg",
                "r_ankle_1",
                "r_ankle_2",
                "r_foot",
                "r_foot_dh_frame",
                "r_sole",
                "r_lower_leg_skin_0",
                "r_lower_leg_skin_10",
                "r_lower_leg_skin_11",
                "r_lower_leg_skin_14",
                "r_lower_leg_skin_15",
                "r_lower_leg_skin_16",
                "r_lower_leg_skin_17",
                "r_lower_leg_skin_19",
                "r_lower_leg_skin_20",
                "r_lower_leg_skin_21",
                "r_lower_leg_skin_28",
                "r_lower_leg_skin_29",
                "r_lower_leg_skin_31",
                "r_lower_leg_skin_32",
                "r_lower_leg_skin_35",
                "r_lower_leg_skin_36",
                "r_lower_leg_skin_37",
                "r_lower_leg_skin_38",
                "r_lower_leg_skin_3",
                "r_lower_leg_skin_41",
                "r_lower_leg_skin_42",
                "r_lower_leg_skin_43",
                "r_lower_leg_skin_46",
                "r_lower_leg_skin_47",
                "r_lower_leg_skin_49",
                "r_lower_leg_skin_4",
                "r_lower_leg_skin_50",
                "r_lower_leg_skin_51",
                "r_lower_leg_skin_52",
                "r_lower_leg_skin_53",
                "r_lower_leg_skin_54",
                "r_lower_leg_skin_55",
                "r_lower_leg_skin_56",
                "r_lower_leg_skin_5",
                "r_lower_leg_skin_60",
                "r_lower_leg_skin_61",
                "r_lower_leg_skin_6",
                "r_lower_leg_skin_9",
                "r_upper_leg_back_contact",
                "r_upper_leg_dh_frame",
                "root_link_imu_frame",
                "torso_1",
                "torso_2",
                "chest",
                "l_shoulder_1",
                "l_shoulder_2",
                "l_shoulder_3",
                "l_upper_arm",
                "l_elbow_1",
                "l_forearm",
                "l_forearm_dh_frame",
                "l_forearm_skin_0",
                "l_forearm_skin_10",
                "l_forearm_skin_11",
                "l_forearm_skin_12",
                "l_forearm_skin_13",
                "l_forearm_skin_14",
                "l_forearm_skin_15",
                "l_forearm_skin_16",
                "l_forearm_skin_17",
                "l_forearm_skin_19",
                "l_forearm_skin_1",
                "l_forearm_skin_22",
                "l_forearm_skin_24",
                "l_forearm_skin_25",
                "l_forearm_skin_28",
                "l_forearm_skin_29",
                "l_forearm_skin_2",
                "l_forearm_skin_3",
                "l_forearm_skin_4",
                "l_forearm_skin_5",
                "l_forearm_skin_6",
                "l_forearm_skin_7",
                "l_forearm_skin_8",
                "l_forearm_skin_9",
                "l_wrist_1",
                "l_hand",
                "l_hand_dh_frame",
                "neck_1",
                "neck_2",
                "head",
                "imu_frame",
                "r_shoulder_1",
                "r_shoulder_2",
                "r_shoulder_3",
                "r_upper_arm",
                "r_elbow_1",
                "r_forearm",
                "r_forearm_dh_frame",
                "r_forearm_skin_0",
                "r_forearm_skin_10",
                "r_forearm_skin_11",
                "r_forearm_skin_12",
                "r_forearm_skin_13",
                "r_forearm_skin_14",
                "r_forearm_skin_15",
                "r_forearm_skin_16",
                "r_forearm_skin_17",
                "r_forearm_skin_19",
                "r_forearm_skin_1",
                "r_forearm_skin_22",
                "r_forearm_skin_24",
                "r_forearm_skin_25",
                "r_forearm_skin_28",
                "r_forearm_skin_29",
                "r_forearm_skin_2",
                "r_forearm_skin_3",
                "r_forearm_skin_4",
                "r_forearm_skin_5",
                "r_forearm_skin_6",
                "r_forearm_skin_7",
                "r_forearm_skin_8",
                "r_forearm_skin_9",
                "r_wrist_1",
                "r_hand",
                "r_hand_dh_frame"};
        }

        std::vector<std::string> leg_eefs()
        {
            return {"r_sole", "l_sole"};
        }

        std::vector<std::string> arm_eefs()
        {
            return {"r_hand", "l_hand"};
        }

        std::string base_link()
        {
            return "root_link";
        }

        std::string urdf_fake_base_link()
        {
            return "base_link";
        }

        std::string head_link()
        {
            return "head";
        }

        std::vector<std::string> joints()
        {
            return {
                "rootJoint",
                "base_fixed_joint",
                "l_hip_pitch",
                "l_hip_roll",
                "l_leg_ft_sensor",
                "l_hip_yaw",
                "l_knee",
                "l_ankle_pitch",
                "l_ankle_roll",
                "l_foot_ft_sensor",
                "l_foot_dh_frame_fixed_joint",
                "l_sole_fixed_joint",
                "l_upper_leg_back_contact_fixed_joint",
                "l_upper_leg_dh_frame_fixed_joint",
                "r_hip_pitch",
                "r_hip_roll",
                "r_leg_ft_sensor",
                "r_hip_yaw",
                "r_knee",
                "r_ankle_pitch",
                "r_ankle_roll",
                "r_foot_ft_sensor",
                "r_foot_dh_frame_fixed_joint",
                "r_sole_fixed_joint",
                "r_lower_leg_skin_0_fixed_joint",
                "r_lower_leg_skin_10_fixed_joint",
                "r_lower_leg_skin_11_fixed_joint",
                "r_lower_leg_skin_14_fixed_joint",
                "r_lower_leg_skin_15_fixed_joint",
                "r_lower_leg_skin_16_fixed_joint",
                "r_lower_leg_skin_17_fixed_joint",
                "r_lower_leg_skin_19_fixed_joint",
                "r_lower_leg_skin_20_fixed_joint",
                "r_lower_leg_skin_21_fixed_joint",
                "r_lower_leg_skin_28_fixed_joint",
                "r_lower_leg_skin_29_fixed_joint",
                "r_lower_leg_skin_31_fixed_joint",
                "r_lower_leg_skin_32_fixed_joint",
                "r_lower_leg_skin_35_fixed_joint",
                "r_lower_leg_skin_36_fixed_joint",
                "r_lower_leg_skin_37_fixed_joint",
                "r_lower_leg_skin_38_fixed_joint",
                "r_lower_leg_skin_3_fixed_joint",
                "r_lower_leg_skin_41_fixed_joint",
                "r_lower_leg_skin_42_fixed_joint",
                "r_lower_leg_skin_43_fixed_joint",
                "r_lower_leg_skin_46_fixed_joint",
                "r_lower_leg_skin_47_fixed_joint",
                "r_lower_leg_skin_49_fixed_joint",
                "r_lower_leg_skin_4_fixed_joint",
                "r_lower_leg_skin_50_fixed_joint",
                "r_lower_leg_skin_51_fixed_joint",
                "r_lower_leg_skin_52_fixed_joint",
                "r_lower_leg_skin_53_fixed_joint",
                "r_lower_leg_skin_54_fixed_joint",
                "r_lower_leg_skin_55_fixed_joint",
                "r_lower_leg_skin_56_fixed_joint",
                "r_lower_leg_skin_5_fixed_joint",
                "r_lower_leg_skin_60_fixed_joint",
                "r_lower_leg_skin_61_fixed_joint",
                "r_lower_leg_skin_6_fixed_joint",
                "r_lower_leg_skin_9_fixed_joint",
                "r_upper_leg_back_contact_fixed_joint",
                "r_upper_leg_dh_frame_fixed_joint",
                "root_link_imu_frame_fixed_joint",
                "torso_pitch",
                "torso_roll",
                "torso_yaw",
                "l_shoulder_pitch",
                "l_shoulder_roll",
                "l_shoulder_yaw",
                "l_arm_ft_sensor",
                "l_elbow",
                "l_wrist_prosup",
                "l_forearm_dh_frame_fixed_joint",
                "l_forearm_skin_0_fixed_joint",
                "l_forearm_skin_10_fixed_joint",
                "l_forearm_skin_11_fixed_joint",
                "l_forearm_skin_12_fixed_joint",
                "l_forearm_skin_13_fixed_joint",
                "l_forearm_skin_14_fixed_joint",
                "l_forearm_skin_15_fixed_joint",
                "l_forearm_skin_16_fixed_joint",
                "l_forearm_skin_17_fixed_joint",
                "l_forearm_skin_19_fixed_joint",
                "l_forearm_skin_1_fixed_joint",
                "l_forearm_skin_22_fixed_joint",
                "l_forearm_skin_24_fixed_joint",
                "l_forearm_skin_25_fixed_joint",
                "l_forearm_skin_28_fixed_joint",
                "l_forearm_skin_29_fixed_joint",
                "l_forearm_skin_2_fixed_joint",
                "l_forearm_skin_3_fixed_joint",
                "l_forearm_skin_4_fixed_joint",
                "l_forearm_skin_5_fixed_joint",
                "l_forearm_skin_6_fixed_joint",
                "l_forearm_skin_7_fixed_joint",
                "l_forearm_skin_8_fixed_joint",
                "l_forearm_skin_9_fixed_joint",
                "l_wrist_pitch",
                "l_wrist_yaw",
                "l_hand_dh_frame_fixed_joint",
                "neck_pitch",
                "neck_roll",
                "neck_yaw",
                "imu_frame_fixed_joint",
                "r_shoulder_pitch",
                "r_shoulder_roll",
                "r_shoulder_yaw",
                "r_arm_ft_sensor",
                "r_elbow",
                "r_wrist_prosup",
                "r_forearm_dh_frame_fixed_joint",
                "r_forearm_skin_0_fixed_joint",
                "r_forearm_skin_10_fixed_joint",
                "r_forearm_skin_11_fixed_joint",
                "r_forearm_skin_12_fixed_joint",
                "r_forearm_skin_13_fixed_joint",
                "r_forearm_skin_14_fixed_joint",
                "r_forearm_skin_15_fixed_joint",
                "r_forearm_skin_16_fixed_joint",
                "r_forearm_skin_17_fixed_joint",
                "r_forearm_skin_19_fixed_joint",
                "r_forearm_skin_1_fixed_joint",
                "r_forearm_skin_22_fixed_joint",
                "r_forearm_skin_24_fixed_joint",
                "r_forearm_skin_25_fixed_joint",
                "r_forearm_skin_28_fixed_joint",
                "r_forearm_skin_29_fixed_joint",
                "r_forearm_skin_2_fixed_joint",
                "r_forearm_skin_3_fixed_joint",
                "r_forearm_skin_4_fixed_joint",
                "r_forearm_skin_5_fixed_joint",
                "r_forearm_skin_6_fixed_joint",
                "r_forearm_skin_7_fixed_joint",
                "r_forearm_skin_8_fixed_joint",
                "r_forearm_skin_9_fixed_joint",
                "r_wrist_pitch",
                "r_wrist_yaw",
                "r_hand_dh_frame_fixed_joint"};
        }

        std::vector<std::string> actuated_joints()
        {
            return {
                // "rootJoint", // we cannot control the floating base
                "l_hip_pitch",
                "l_hip_roll",
                "l_hip_yaw",
                "l_knee",
                "l_ankle_pitch",
                "l_ankle_roll",
                "r_hip_pitch",
                "r_hip_roll",
                "r_hip_yaw",
                "r_knee",
                "r_ankle_pitch",
                "r_ankle_roll",
                "torso_pitch",
                "torso_roll",
                "torso_yaw",
                "l_shoulder_pitch",
                "l_shoulder_roll",
                "l_shoulder_yaw",
                "l_elbow",
                "l_wrist_prosup",
                "l_wrist_pitch",
                "l_wrist_yaw",
                "neck_pitch",
                "neck_roll",
                "neck_yaw",
                "r_shoulder_pitch",
                "r_shoulder_roll",
                "r_shoulder_yaw",
                "r_elbow",
                "r_wrist_prosup",
                "r_wrist_pitch",
                "r_wrist_yaw"};
        }
    } // namespace model
} // namespace icub