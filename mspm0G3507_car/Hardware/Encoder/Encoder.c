// /*
// 外部中断需要占用4个GPIO，PORT为Encoder_PORT，PIN定义为Encoder_A_PIN，Encoder_B_PIN，Encoder_C_PIN，Encoder_D_PIN
// 注：这里有些引脚采集外部中断的数据会有问题，建议使用默认B6，B7，B8，B9引脚
// 使用前要使能编码器中断NVIC_EnableIRQ(Encoder_INT_IRQN);//编码器中断使能

// *  只需要调用MEASURE_MOTORS_SPEED()函数，且Motor1_Speed和Motor2_Speed为电机实际速度
// */
// #include "Encoder.h"
// #include "mpu6050.h"
// #include "key.h"

// int32_t Motor1_Encoder_Value=0;
// int32_t Motor2_Encoder_Value=0;

// float Motor1_Speed = 0;
// float Motor2_Speed = 0;

// //外部中断读取编码器的值
// void GROUP1_IRQHandler(void){
// 	if(DL_Interrupt_getStatusGroup(DL_INTERRUPT_GROUP_1,DL_INTERRUPT_GROUP1_GPIOB)){
// 		uint32_t Encoder_GPIO_Int = DL_GPIO_getEnabledInterruptStatus(Encoder_PORT,Encoder_A_PIN | Encoder_B_PIN | Encoder_C_PIN | Encoder_D_PIN);
// 		//通道1 左轮A相
// 		if ((Encoder_GPIO_Int & Encoder_A_PIN) == Encoder_A_PIN){
// 			DL_GPIO_clearInterruptStatus(Encoder_PORT, Encoder_A_PIN);
// 			if(Read_Encoder_A == 1){ //上升沿
// 				if (Read_Encoder_B == 0){
// 					Motor1_Encoder_Value++;
// 				}
// 				else if(Read_Encoder_B == 1){
// 					Motor1_Encoder_Value--;
// 				}
// 			}
// 			else if(Read_Encoder_A == 0){//下降沿
// 				if (Read_Encoder_B == 0){
// 					Motor1_Encoder_Value--;
// 				}
// 				else if(Read_Encoder_B == 1){
// 					Motor1_Encoder_Value++;
// 				}
// 			}
// 		}		
// 		//通道2 左轮B相
// 		if ((Encoder_GPIO_Int & Encoder_B_PIN) == Encoder_B_PIN){
// 			DL_GPIO_clearInterruptStatus(Encoder_PORT,Encoder_B_PIN);
// 			if(Read_Encoder_B == 1){ //上升沿
// 				if (Read_Encoder_A == 0){
// 					Motor1_Encoder_Value--;
// 				}
// 				else if (Read_Encoder_A == 1){
// 					Motor1_Encoder_Value++;
// 				}
// 			}
// 			else if(Read_Encoder_B == 0){//下降沿
// 				if (Read_Encoder_A == 0){
// 					Motor1_Encoder_Value++;
// 				}
// 				else if (Read_Encoder_A == 1){
// 					Motor1_Encoder_Value--;
// 				}
// 			}
// 		}
// 		//通道3 右轮A相		
// 		if ((Encoder_GPIO_Int & Encoder_C_PIN) == Encoder_C_PIN){
// 			DL_GPIO_clearInterruptStatus(Encoder_PORT, Encoder_C_PIN);
// 			if(Read_Encoder_C == 1){ //上升沿
// 				if (Read_Encoder_D  == 0){
// 					Motor2_Encoder_Value++;
// 				}
// 				else if (Read_Encoder_D  == 1){
// 					Motor2_Encoder_Value--;
// 				}
// 			}
// 			else if(Read_Encoder_C == 0){//下降沿
// 				if (Read_Encoder_D  == 0){
// 					Motor2_Encoder_Value--;
// 				}
// 				else if (Read_Encoder_D  == 1){
// 					Motor2_Encoder_Value++;
// 				}
// 			}
// 		}		
// 		//通道4 右轮B相
// 		if ((Encoder_GPIO_Int & Encoder_D_PIN) == Encoder_D_PIN){
// 			DL_GPIO_clearInterruptStatus(Encoder_PORT,Encoder_D_PIN);
// 			if(Read_Encoder_D == 1){ //上升沿
// 				if (Read_Encoder_C  == 0){
// 					Motor2_Encoder_Value--;
// 				}
// 				else if (Read_Encoder_C  == 1){
// 					Motor2_Encoder_Value++;
// 				}
// 			}
// 			else if(Read_Encoder_D == 0){//下降沿
// 				if (Read_Encoder_C  == 0){
// 					Motor2_Encoder_Value++;
// 				}
// 				else if (Read_Encoder_C  == 1){
// 					Motor2_Encoder_Value--;
// 				}
// 			}
// 		}
// 	}
// }
// //编码器1计算速度
// void Motor1_Get_Speed(void){
//     short Encoder_TIM = 0;
//     float Speed = 0;
//     Encoder_TIM= Motor1_Encoder_Value;
//     Motor1_Encoder_Value=0;
//     Speed =(float)Encoder_TIM/(CC)*PI*RR;//计算速度
//     Motor1_Speed = Speed;
// }
// //编码器2计算速度
// void Motor2_Get_Speed(void){
//     short Encoder_TIM = 0;
//     float Speed = 0;
//     Encoder_TIM= Motor2_Encoder_Value;
//     Motor2_Encoder_Value=0;
//     Speed =(float)Encoder_TIM/(CC)*PI*RR;//计算速度
//     Motor2_Speed = -Speed;
// }
// float Motor1_Lucheng,Motor2_Lucheng;
// float Measure_Distance = 0;
// //测量所有电机速度
// void MEASURE_MOTORS_SPEED(void){
// 	Motor1_Get_Speed();Motor2_Get_Speed();
	
// 	Motor1_Lucheng += Motor1_Speed*SAMPLE_TIME;//路程累计
// 	Motor2_Lucheng += Motor2_Speed*SAMPLE_TIME;//路程累计
// 	Measure_Distance = Motor1_Lucheng/2.0 +Motor2_Lucheng/2.0;
	
// 	if(Measure_Distance>10000)
// 	{
// 		Measure_Distance = 0;
// 		Motor1_Lucheng = 0;
// 		Motor2_Lucheng = 0;
		
// 	}
// }
