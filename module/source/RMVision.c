
/////*
////@author: CodeAlan  华南师大Vanguard战队
////*/
////// 弹道解算
////// 只考虑水平方向的空气阻力

////#include <arm_math.h>
////#include <math.h>
////#include <stdio.h>

////#include <cmsis_os.h>

////#include "SolveTrajectory.h"
////#include "vision.h"
////#include "beep.h"

////struct SolveTrajectoryParams st;
////struct tar_pos tar_position[4];
////float stored_y = 0.0f; //放全局里面方便调试查看
////float static_z = 0.21f;//增大是往下
////float static_yaw = 0.000f; // One degree = 0.01745 Radian，增大往左



////float b=0;
////_Bool rmvisionSolvingGimbal = 0;

/////*
////@brief 初始化
////@param pitch:rad
////@param yaw:rad
////@param v:m/s
////@param k:弹道系数
////*/
////void GimbalControlInit(float pitch, float yaw, float tar_yaw, float v_yaw, float r1, float r2, float z2, uint8_t armor_type, float v, float k)
////{
////    st.current_pitch = pitch;
////    st.current_yaw = yaw;
////    st.current_v = v;
////    st.k = k;
////    st.tar_yaw = tar_yaw;
////    st.v_yaw = v_yaw;
////    st.r1 = r1;
////    st.r2 = r2;
////    st.dz = z2; // FIXME: z2? dz?
////    st.armor_type = armor_type;
////}

////#if 1 // 0 时为孙岩的单方向模型，1 时为Alan开源代码里的单方向模型
////float t = 0.20f; // 飞行时间

/////*
////@brief 弹道模型
////@param x:m 距离
////@param v:m/s 速度
////@param angle:rad 角度
////@return y:m
////*/
////float GimbalControlBulletModel(float x, float v, float angle)
////{
////    float y;
////    t = (float)((exp(st.k * x) - 1) / (st.k * v * arm_cos_f32(angle)));
////    y = (float)(v * arm_sin_f32(angle) * t - GRAVITY * t * t / 2.0f);
////    return y;
////}


/////*
////@brief pitch轴解算
////@param x:m 距离
////@param y:m 高度
////@param v:m/s
////@return angle_pitch:rad
////*/
////float GimbalControlGetPitch(float x, float y, float v)
////{
////    float y_temp, y_actual, dy;
////    float angle_pitch;
////		stored_y = y;
////    y_temp = y;
////    // iteration
////    int i = 0;
////    for (i = 0; i < 20; i++)
////    {
////        angle_pitch = (float)atan2(y_temp, x); // rad
////        y_actual = GimbalControlBulletModel(x, v, angle_pitch);
////        dy = 0.3f*(y - y_actual);
////        y_temp = y_temp + dy;
////        if (fabsf(dy) < 0.00001f)
////        {
////            break;
////        }
////    }
////		
////		y_temp = stored_y;// 假装读取不要让编译器优化掉stored_y
////		
////    return angle_pitch;
////}
////#else

////static float theta = .0f, v0x = .0f, v0now = .0f, t = .0f, ek = 1.0f;
////static float dk = .0f, vk = .0f, dr = .0f, hr;
////static const float k = 0.00556, dt = 0.0005;

////float Trajectory_timefd(float v0x, float t1)
////{
////	float tmp_what = k * v0x * t1 + 1.0f;
////	dk = 1.0f / k * log(tmp_what);
////	vk = v0x / (tmp_what);
////	return (dr - dk) / vk;
////}

////void Trajectory_time(float v0x, float dr)
////{
////	for (int iter = 0; iter < 20 && fabsf(dr - dk) > 0.01f; iter++)
////	{
////		t += Trajectory_timefd(v0x, t);
////	}
////}

////float Trajectory_anglefd(float t2, float theta)
////{
////	float hk = v0now * sin(theta) * t2 - 4.9f * t2 * t2;
////	return hr - hk;
////}

////float GimbalControlGetPitch(float x, float y, float v)
////{
////	float dr = sqrt(x * x + y * y);
////	hr = y;
////	v0now = (v == 0.0f) ? 15.7f : v;
////	
////	for (int iter = 0; iter < 40 && fabsf(ek) > 0.01f; iter++)
////	{
////		v0x = v0now * arm_cos_f32(theta);
////		Trajectory_time(v0x, dr);
////		ek = Trajectory_anglefd(t, theta);
////		theta += ek * 0.04f;
////	}
////	
////	return theta;
////}

////#endif

/////*
////@brief 世界坐标系转换到云台坐标系
////@param xw:ROS坐标系下的x
////@param yw:ROS坐标系下的y
////@param zw:ROS坐标系下的z
////@param vxw:ROS坐标系下的vx
////@param vyw:ROS坐标系下的vy
////@param vzw:ROS坐标系下的vz
////@param bias_time:固定时间延迟偏置 单位ms
////@param pitch:rad  传出pitch
////@param yaw:rad    传出yaw
////@param aim_x:传出aim_x  打击目标的x
////@param aim_y:传出aim_y  打击目标的y
////@param aim_z:传出aim_z  打击目标的z
////*/
////void GimbalControlTransform(float xw, float yw, float zw,
////                            float vxw, float vyw, float vzw,
////                            int bias_time, float *pitch, float *yaw,
////                            float *aim_x, float *aim_y, float *aim_z)
////{
////    float s_static = 0.10; //枪口前推的距离
////	  float z_static = static_z;//0.08f;//0.16; //yaw轴电机到枪口水平面的垂直距离 // FIXME: 是电机到枪口还是相机到枪口？

////    // 线性预测
////    float timeDelay = bias_time/1000.0 + t;
////    st.tar_yaw += st.v_yaw * timeDelay;

////    //计算四块装甲板的位置
////	int use_1 = 1;
////	int i = 0;
////    int idx = 0; // 选择的装甲板
////    //armor_type = 1 为平衡步兵
////	
////    if (st.armor_type == 1) {
////        for (i = 0; i<2; i++) {
////            float tmp_yaw = st.tar_yaw + i * PI;
////            float r = st.r1;
////            tar_position[i].x = xw - r*cos(tmp_yaw);
////            tar_position[i].y = yw - r*sin(tmp_yaw);
////            tar_position[i].z = zw;
////            tar_position[i].yaw = st.tar_yaw + i * PI;
////        }

////        float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);

////        //因为是平衡步兵 只需判断两块装甲板即可
////        float temp_yaw_diff = fabsf(*yaw - tar_position[1].yaw);
////        if (temp_yaw_diff < yaw_diff_min)
////        {
////            yaw_diff_min = temp_yaw_diff;
////            idx = 1;
////        }


////    } else {

////        for (i = 0; i<4; i++) {
////            float tmp_yaw = st.tar_yaw + i * PI/2.0f;
////            float r = use_1 ? st.r1 : st.r2;
////            tar_position[i].x = xw - r*cos(tmp_yaw);
////            tar_position[i].y = yw - r*sin(tmp_yaw);
////            tar_position[i].z = use_1 ? zw : st.dz + zw;
////            tar_position[i].yaw = st.tar_yaw + i * PI/2.0f;
////            use_1 = !use_1;
////        }

////            //2种常见决策方案：
////            //1.计算枪管到目标装甲板yaw最小的那个装甲板
////            //2.计算距离最近的装甲板


////        

////            //计算枪管到目标装甲板yaw最小的那个装甲板
//////        float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);
//////        for (i = 1; i<4; i++) {
//////            float temp_yaw_diff = fabsf(*yaw - tar_position[i].yaw);
//////            if (temp_yaw_diff < yaw_diff_min)
//////            {
//////                yaw_diff_min = temp_yaw_diff;
//////                idx = i;
//////            }
//////						b= sqrt(tar_position[0].x * tar_position[0].x + tar_position[0].y * tar_position[0].y);
//////        }

////    }
////		float detectx[4];
////		float detecty[4];
////		float detectz[4];
////					for(int i=0;i<4;i++)
////		{
////		detectz[i] = tar_position[i].z + vzw * timeDelay;
////    detectx[i]= tar_position[i].x + vxw * timeDelay;
////    detecty[i]= tar_position[i].y + vyw * timeDelay;
////			
////		}


//////            //计算距离最近的装甲板
////        	float dis_diff_min = sqrt(detectx[0] * detectx[0] + detecty[0] * detecty[0]);
////        	for (i = 1; i<4; i++)
////        	{
////        		float temp_dis_diff = sqrt(detectx[i] * detectx[i] + detecty[i] * detecty[i]);
////        		if (temp_dis_diff < dis_diff_min)
////        		{
////        			dis_diff_min = temp_dis_diff;
////        			idx = i;
////        		}
////        	}
////	  *aim_z = detectz[idx];
////    *aim_x = detectx[idx];
////    *aim_y = detecty[idx];

////    *pitch = GimbalControlGetPitch(sqrt((*aim_x) * (*aim_x) + (*aim_y) * (*aim_y)) + s_static,
////            tar_position[idx].z - z_static, st.current_v);
////    *yaw = (float)(atan2(*aim_y, *aim_x)) + (static_yaw * (*aim_x) * 0.46);
////				
////}

////// 从坐标轴正向看向原点，逆时针方向为正
////void OS_RmVisionCallBack(void const * argument)
////{
////	while (1) {
////		// 无限等待直到USB CDC的ISR唤醒
////     while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
////        {
////        }
////		rmvisionSolvingGimbal = 1;
////		Vision_ParseData();
////		rmvisionSolvingGimbal = 0;
////		//Beep_PlayNotes((Note[]){{T_M3,D_Sixteenth},{T_M3,D_Sixteenth},{T_None,D_Quarter}},3);
////	}
////}


















///*
//@author: CodeAlan  华南师大Vanguard战队
//*/
//// 弹道解算
//// 只考虑水平方向的空气阻力

//#include <arm_math.h>
//#include <math.h>
//#include <stdio.h>

//#include <cmsis_os.h>

//#include "SolveTrajectory.h"
//#include "vision.h"
//#include "beep.h"


//struct SolveTrajectoryParams st;
//struct tar_pos tar_position[4];
//float stored_y = 0.0f; //放全局里面方便调试查看
//float static_z = 0.20f;//增大是往下
//float static_yaw = 0.000f; // One degree = 0.01745 Radian，增大往左

//_Bool rmvisionSolvingGimbal = 0;

///*
//@brief 初始化
//@param pitch:rad
//@param yaw:rad
//@param v:m/s
//@param k:弹道系数
//*/
//void GimbalControlInit(float pitch, float yaw, float tar_yaw, float v_yaw, float r1, float r2, float z2, uint8_t armor_type, float v, float k)
//{
//    st.current_pitch = pitch;
//    st.current_yaw = yaw;
//    st.current_v = v;
//    st.k = k;
//    st.tar_yaw = tar_yaw;
//    st.v_yaw = v_yaw;
//    st.r1 = r1;
//    st.r2 = r2;
//    st.dz = z2; // FIXME: z2? dz?
//    st.armor_type = armor_type;
//} 

//#if 1 // 0 时为孙岩的单方向模型，1 时为Alan开源代码里的单方向模型
//float t = 0.20f; // 飞行时间

///*
//@brief 弹道模型
//@param x:m 距离
//@param v:m/s 速度
//@param angle:rad 角度
//@return y:m
//*/
//float GimbalControlBulletModel(float x, float v, float angle)
//{
//    float y;
//    t = (float)((exp(st.k * x) - 1) / (st.k * v * arm_cos_f32(angle)));
//    y = (float)(v * arm_sin_f32(angle) * t - GRAVITY * t * t / 2.0f);
//    return y;
//}


///*
//@brief pitch轴解算
//@param x:m 距离
//@param y:m 高度
//@param v:m/s
//@return angle_pitch:rad
//*/
//float GimbalControlGetPitch(float x, float y, float v)
//{
//    float y_temp, y_actual, dy;
//    float angle_pitch;
//		stored_y = y;
//    y_temp = y;
//    // iteration
//    int i = 0;
//    for (i = 0; i < 20; i++)
//    {
//        angle_pitch = (float)atan2(y_temp, x); // rad
//        y_actual = GimbalControlBulletModel(x, v, angle_pitch);
//        dy = 0.3f*(y - y_actual);
//        y_temp = y_temp + dy;
//        if (fabsf(dy) < 0.00001f)
//        {
//            break;
//        }
//    }
//		
//		y_temp = stored_y;// 假装读取不要让编译器优化掉stored_y
//		
//    return angle_pitch;
//}
//#else

//static float theta = .0f, v0x = .0f, v0now = .0f, t = .0f, ek = 1.0f;
//static float dk = .0f, vk = .0f, dr = .0f, hr;
//static const float k = 0.00556, dt = 0.0005;

//float Trajectory_timefd(float v0x, float t1)
//{
//	float tmp_what = k * v0x * t1 + 1.0f;
//	dk = 1.0f / k * log(tmp_what);
//	vk = v0x / (tmp_what);
//	return (dr - dk) / vk;
//}

//void Trajectory_time(float v0x, float dr)
//{
//	for (int iter = 0; iter < 20 && fabsf(dr - dk) > 0.01f; iter++)
//	{
//		t += Trajectory_timefd(v0x, t);
//	}
//}

//float Trajectory_anglefd(float t2, float theta)
//{
//	float hk = v0now * sin(theta) * t2 - 4.9f * t2 * t2;
//	return hr - hk;
//}

//float GimbalControlGetPitch(float x, float y, float v)
//{
//	float dr = sqrt(x * x + y * y);
//	hr = y;
//	v0now = (v == 0.0f) ? 15.7f : v;
//	
//	for (int iter = 0; iter < 40 && fabsf(ek) > 0.01f; iter++)
//	{
//		v0x = v0now * arm_cos_f32(theta);
//		Trajectory_time(v0x, dr);
//		ek = Trajectory_anglefd(t, theta);
//		theta += ek * 0.04f;
//	}
//	
//	return theta;
//}

//#endif

///*
//@brief 世界坐标系转换到云台坐标系
//@param xw:ROS坐标系下的x
//@param yw:ROS坐标系下的y
//@param zw:ROS坐标系下的z
//@param vxw:ROS坐标系下的vx
//@param vyw:ROS坐标系下的vy
//@param vzw:ROS坐标系下的vz
//@param bias_time:固定时间延迟偏置 单位ms
//@param pitch:rad  传出pitch
//@param yaw:rad    传出yaw
//@param aim_x:传出aim_x  打击目标的x
//@param aim_y:传出aim_y  打击目标的y
//@param aim_z:传出aim_z  打击目标的z
//*/
//void GimbalControlTransform(float xw, float yw, float zw,
//                            float vxw, float vyw, float vzw,
//                            int bias_time, float *pitch, float *yaw,
//                            float *aim_x, float *aim_y, float *aim_z)
//{
//    float s_static = 0.10; //枪口前推的距离
//	  float z_static = static_z;//0.08f;//0.16; //yaw轴电机到枪口水平面的垂直距离 // FIXME: 是电机到枪口还是相机到枪口？

//    // 线性预测
//    float timeDelay = bias_time/1000.0 + t;
//    st.tar_yaw += st.v_yaw * timeDelay;

//    //计算四块装甲板的位置
//	int use_1 = 1;
//	int i = 0;
//    int idx = 0; // 选择的装甲板
//    //armor_type = 1 为平衡步兵
//    if (st.armor_type == 1) {
//        for (i = 0; i<2; i++) {
//            float tmp_yaw = st.tar_yaw + i * PI;
//            float r = st.r1;
//            tar_position[i].x = xw - r*cos(tmp_yaw);
//            tar_position[i].y = yw - r*sin(tmp_yaw);
//            tar_position[i].z = zw;
//            tar_position[i].yaw = st.tar_yaw + i * PI;
//        }

//        float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);

//        //因为是平衡步兵 只需判断两块装甲板即可
//        float temp_yaw_diff = fabsf(*yaw - tar_position[1].yaw);
//        if (temp_yaw_diff < yaw_diff_min)
//        {
//            yaw_diff_min = temp_yaw_diff;
//            idx = 1;
//        }


//    } else {

//        for (i = 0; i<4; i++) {
//            float tmp_yaw = st.tar_yaw + i * PI/2.0f;
//            float r = use_1 ? st.r1 : st.r2;
//            tar_position[i].x = xw - r*cos(tmp_yaw);
//            tar_position[i].y = yw - r*sin(tmp_yaw);
//            tar_position[i].z = use_1 ? zw : st.dz + zw;
//            tar_position[i].yaw = st.tar_yaw + i * PI/2.0f;
//            use_1 = !use_1;
//        }

//            //2种常见决策方案：
//            //1.计算枪管到目标装甲板yaw最小的那个装甲板
//            //2.计算距离最近的装甲板

////		//计算距离最近的装甲板
////	float dis_diff_min = sqrt(tar_position[0].x * tar_position[0].x + tar_position[0].y * tar_position[0].y);
////	int idx = 0;
////	for (i = 1; i<4; i++)
////	{
////		float temp_dis_diff = sqrt(tar_position[i].x * tar_position[0].x + tar_position[i].y * tar_position[0].y);
////		if (temp_dis_diff < dis_diff_min)
////		{
////			dis_diff_min = temp_dis_diff;
////			idx = i;
////		}
////	}


//            //计算枪管到目标装甲板yaw最小的那个装甲板
//        float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);
//        for (i = 1; i<4; i++) {
//            float temp_yaw_diff = fabsf(*yaw - tar_position[i].yaw);
//            if (temp_yaw_diff < yaw_diff_min)
//            {
//                yaw_diff_min = temp_yaw_diff;
//                idx = i;
//            }
//        }

//    }

//	

//    *aim_z = tar_position[idx].z + vzw * timeDelay;
//    *aim_x = tar_position[idx].x + vxw * timeDelay;
//    *aim_y = tar_position[idx].y + vyw * timeDelay;

//    *pitch = GimbalControlGetPitch(sqrt((*aim_x) * (*aim_x) + (*aim_y) * (*aim_y)) + s_static,
//            tar_position[idx].z - z_static, st.current_v);
//    *yaw = (float)(atan2(*aim_y, *aim_x)) + (static_yaw * (*aim_x) * 0.46);

//}

//// 从坐标轴正向看向原点，逆时针方向为正
//void OS_RmVisionCallBack(void const * argument)
//{
//	while (1) {
//		// 无限等待直到USB CDC的ISR唤醒
//     while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
//        {
//        }
//		rmvisionSolvingGimbal = 1;
//		Vision_ParseData();
//		rmvisionSolvingGimbal = 0;
//		//Beep_PlayNotes((Note[]){{T_M3,D_Sixteenth},{T_M3,D_Sixteenth},{T_None,D_Quarter}},3);
//	}
//}

