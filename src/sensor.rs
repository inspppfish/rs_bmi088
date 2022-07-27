use core::f32::consts::PI;
use libm::sqrt;
use crate::math::Quaternion;
#[derive(Clone, Debug)]
pub struct Sensor {
    pub acc_x: f32,
    pub acc_y: f32,
    pub acc_z: f32,
    pub gyro_x: f32,
    pub gyro_y: f32,
    pub gyro_z: f32,
    pub gyro_error_x: f32,
    pub gyro_error_y: f32,
    pub gyro_error_z: f32,
    pub atti_num_q: Quaternion,
    pub yaw: f32,
    pub pitch: f32,
    pub roll: f32,
}

impl Sensor {
    pub fn new () -> Self {
        Sensor {
            acc_x: 0_f32,
            acc_y: 0_f32,
            acc_z: 0_f32,
            gyro_x: 0_f32,
            gyro_y: 0_f32,
            gyro_z: 0_f32,
            gyro_error_x: 0_f32,
            gyro_error_y: 0_f32,
            gyro_error_z: 0_f32,
            atti_num_q: Quaternion {
                q0: 1.0,
                q1: 0.0,
                q2: 0.0,
                q3: 0.0,
            },
            yaw: 0_f32,
            pitch: 0_f32,
            roll: 0_f32,
        }
    }

    pub fn attitude_solution(&mut self, dt:&f32) -> Result<(), AttitudeError> {
        use libm::{sqrtf, asinf, atan2f};
        struct Value {
            pub x: f32,
            pub y: f32,
            pub z: f32,
        }
        const KP: f32 = 0.8;
        const KI: f32 = 0.0003;

        // 提取等效旋转矩阵中的重力分量
        let virtual_gravity = Value {
            x: 2.0*(self.atti_num_q.q1 * self.atti_num_q.q3 - self.atti_num_q.q0 * self.atti_num_q.q2),
            y: 2.0*(self.atti_num_q.q0 * self.atti_num_q.q1 + self.atti_num_q.q2 * self.atti_num_q.q3),
            z: 1.0 - 2.0*(self.atti_num_q.q1*self.atti_num_q.q1 + self.atti_num_q.q2*self.atti_num_q.q2),
        };

        // 加速度归一化
        let norm_acc = 1.0/sqrtf(self.acc_x*self.acc_x + self.acc_y*self.acc_y + self.acc_z*self.acc_z);
        let virtual_acc = Value {
            x: self.acc_x * norm_acc,
            y: self.acc_y * norm_acc,
            z: self.acc_z * norm_acc,
        };

        //向量差乘得出的值
        let virtual_acc_gravity = Value {
            x: virtual_acc.y * virtual_gravity.z - virtual_acc.z * virtual_gravity.y,
            y: virtual_acc.z * virtual_gravity.x - virtual_acc.x * virtual_gravity.z,
            z: virtual_acc.x * virtual_gravity.y - virtual_acc.y * virtual_gravity.x,
        };

        ////再做加速度积分补偿角速度的补偿值
        self.gyro_error_x += KI * virtual_acc_gravity.x;
        self.gyro_error_y += KI * virtual_acc_gravity.y;
        self.gyro_error_z += KI * virtual_acc_gravity.z;

        //角速度融合加速度积分补偿值
        let virtual_gyro = Value {
            x: self.gyro_x + KP * virtual_acc_gravity.x +  self.gyro_error_x,
            y: self.gyro_y + KP * virtual_acc_gravity.y +  self.gyro_error_y,
            z: self.gyro_z + KP * virtual_acc_gravity.z +  self.gyro_error_z,
        };

        // 一阶龙格库塔法, 更新四元数
        self.atti_num_q.q0 += ( - self.atti_num_q.q1 * virtual_gyro.x
                                - self.atti_num_q.q2 * virtual_gyro.y
                                - self.atti_num_q.q3 * virtual_gyro.z
        ) * dt/2.0;
        self.atti_num_q.q1 += (   self.atti_num_q.q0 * virtual_gyro.x
                                - self.atti_num_q.q3 * virtual_gyro.y
                                + self.atti_num_q.q2 * virtual_gyro.z
        ) * dt/2.0;
        self.atti_num_q.q2 += (   self.atti_num_q.q3 * virtual_gyro.x
                                + self.atti_num_q.q0 * virtual_gyro.y
                                - self.atti_num_q.q1 * virtual_gyro.z
        ) * dt/2.0;
        self.atti_num_q.q3 += ( - self.atti_num_q.q2 * virtual_gyro.x
                                + self.atti_num_q.q1 * virtual_gyro.y
                                + self.atti_num_q.q0 * virtual_gyro.z
        ) * dt/2.0;

        // 四元数归一化
        let normal_quat = 1.0/sqrtf(self.atti_num_q.q0 * self.atti_num_q.q0
                                    +self.atti_num_q.q1 * self.atti_num_q.q1
                                    +self.atti_num_q.q2 * self.atti_num_q.q2
                                    +self.atti_num_q.q3 * self.atti_num_q.q3);
        self.atti_num_q.q0 *= normal_quat;
        self.atti_num_q.q1 *= normal_quat;
        self.atti_num_q.q2 *= normal_quat;
        self.atti_num_q.q3 *= normal_quat;

        // 四元数转欧拉角
        const  RTA: f32 = 57.2957795;
        if self.gyro_z > 3.0 || self.gyro_z < -3.0 {
            self.yaw += self.gyro_z; // yaw 积分
        }
        self.pitch = asinf(2.0 * self.atti_num_q.q0 * self.atti_num_q.q2
                            - 2.0 * self.atti_num_q.q1 * self.atti_num_q.q3) * RTA;
        self.roll= atan2f(2.0 * self.atti_num_q.q2 * self.atti_num_q.q3
                           + 2.0 * self.atti_num_q.q0 * self.atti_num_q.q1,
                          1.0 - 2.0 * self.atti_num_q.q1 * self.atti_num_q.q1
                              - 2.0 * self.atti_num_q.q2 * self.atti_num_q.q2) * RTA;
        Ok(())
    }
}

pub enum AttitudeError {
    AngleError,
}