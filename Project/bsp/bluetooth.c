///*******************************************************************************************************************/
///***Include***/
////{
//#include "zf_common_headfile.h"
//#include "bsp.h"
//#include "string.h"
////}
///*******************************************************************************************************************/
///***Functions***/
////{									
//
//uint8 	receive_fifo_buffer[128] 	= {0}; 				// fifo数据存储位置
//uint8 	receive_buffer[64] 	= {0}; 					// 单包数据存放位置
//
//uint8 	receive_data[64]	= {0};			        	// 解析完成的数据存放位置
// 									 
//uint8 	one_bag_num = 0;					        // 单包数据ID
//uint8   test = 0;
//uint8 	data_arry_temp[64] = {0};
//uint8   mutch_count = 0;
//uint8   mutch_flag = 0;
//uint8   mutch_data[2] = {0,0};
//lpuart_handle_t     buletooth_g_lpuartHandle;
////-------------------------------------------------------------------------------------------------------------------
//// 函数简介     Hex 转字符串
//// 参数说明     *str            字符串指针
//// 参数说明     number          传入的数据
//// 返回参数     void
//// 使用示例     func_hex_to_str(data_buffer, 0x11);                             // 结果输出 data_buffer = "0x11"
//// 备注信息     
////-------------------------------------------------------------------------------------------------------------------
//void func_hex_to_str (char *str, uint32 number)
//{
//    assert(str != NULL);
//    const char hex_index[16] = {
//        '0', '1', '2', '3',
//        '4', '5', '6', '7',
//        '8', '9', 'A', 'B',
//        'C', 'D', 'E', 'F'};
//    int8 data_temp[12];                                                         // 缓冲区
//    uint8 bit = 0;                                                              // 数字位数
//
//    *str++ = '0';
//    *str++ = 'x';
//    do
//    {
//        if(NULL == str)
//        {
//            break;
//        }
//
//        if(0 == number)                                                         // 这是个 0
//        {
//            *str = '0';
//            break;
//        }
//
//        while(0 != number)                                                      // 循环直到数值归零
//        {
//            data_temp[bit ++] = (number & 0xF);                                 // 倒序将数值提取出来
//            number >>= 4;                                                       // 削减被提取的个位数
//        }
//        while(0 != bit)                                                         // 提取的数字个数递减处理
//        {
//            *str ++ = hex_index[data_temp[bit - 1]];                            // 将数字从倒序数组中倒序取出 变成正序放入字符串
//            bit --;
//        }
//    }while(0);
//}
////-------------------------------------------------------------------------------------------------------------------
//// 函数简介     蓝牙转串口模块 发送函数
//// 参数说明     buff             需要发送的数据地址
//// 返回参数     len              发送长度
//// 使用示例     uint32           剩余未发送的字节数
//// 使用示例     bluetooth_ch9141_send_buff(buff, 16);
//// 备注信息     
////-------------------------------------------------------------------------------------------------------------------
//uint32 bluetooth_ch9141_send_buff (const uint8 *buff, uint32 len)
//{
//    assert(buff != NULL);
//    uint16 time_count = 0;
//    while(0 != len)
//    {
//        if(!gpio_get(D26))                           // 如果RTS为低电平 则继续发送数据
//        {
//            if(30 <= len)                                                       // 数据分 30byte 每包发送
//            {
//                uart_putbuff(UART_8, (uint8 *)buff, 30);            // 发送数据
//                buff += 30;                                                     // 地址偏移
//                len -= 30;                                                      // 数量
//                time_count = 0;
//            }
//            else                                                                // 不足 30byte 的数据一次性发送完毕
//            {
//                uart_putbuff(UART_8, (uint8 *)buff, len);           // 发送数据
//                len = 0;
//                break;
//            }
//        }
//        else                                                                    // 如果RTS为高电平 则模块忙
//        {
//            if(500 <= (++ time_count))               // 超出了最大等待时间
//            {
//                break;                                                          // 退出发送
//            }
//            systick_delay_ms(1);
//        }
//    }
//    return len;
//}
//void bluetooth_callback(void)
//{
// 
//	uint8 	data_temp = 0;
//	uint32 	receive_count 		= 0;
//	
//	uart_query(UART_8, &data_temp);            // 读取串口数据
//	
//     data_arry_temp[test]=data_temp;
//	test++;
//        test%=64;
//	if(data_temp == '\n')											// 接收到帧尾数据
//	{
//          test=0;
//		if(one_bag_receive_flag == 0)								// 如果没有在解析数据，那么将数据读取到缓冲区，并且置位解析标志位
//		{
//                      
//			receive_count = strlen((char *)data_arry_temp);	
//                        for(int n=0;n<receive_count;n++)
//                        {
//                        receive_buffer[n]=data_arry_temp[n];
//                        
//                        }
//			//fifo_read_buffer(&bluetooth_receive_fifo, &receive_buffer, &receive_count, FIFO_READ_AND_CLEAN);      // 读取 FIFO 数据到缓冲区	
//			one_bag_receive_flag = 1;
//		}
//		else
//		{
//                  receive_count = strlen((char *)data_arry_temp);	
//                  memset(data_arry_temp,0,receive_count);	
//                }
//	}
//}
//uint8 ppp=0;
//void bluetooth_data_parse(void)
//{
//              // gpio_init(B9,GPO,0,GPIO_PIN_CONFIG);  
//	uint8 verify_data[4] 		= {0};
//	uint8 data_temp 			= 0;
//	uint8 data_count 			= 0;
//        uint8 sum_check[4]         ={0};
//        
//	data_count = strlen((char *)receive_buffer);					// 统计这包数据长度
//	
//	memset(receive_data, 0, sizeof(receive_data));					// 清空上次接收到的数据
//	//PRINTF("aaaaaaaaaaaaaaa%s\r\n",(char *)receive_buffer);
//	for(uint8 i = 3; i < data_count; i ++) 							// 计算所有数据和
//	{
//		data_temp +=  receive_buffer[i];
//                
//	}
//        //PRINTF("%c\r\n",data_temp);
//	func_hex_to_str((char *)verify_data, data_temp);
//        //PRINTF("%s,%c,%c\r\n",verify_data,receive_buffer[0],receive_buffer[1]);
//        
//	if(data_temp < 16) 												// func_hex_to_str函数转换0x0F的结果是0xF  因此需要移位
//	{
//		verify_data[3] = verify_data[2];
//		verify_data[2] = 0x30;
//                
//	}
//        data_temp=data_temp-receive_buffer[3]-receive_buffer[4]-receive_buffer[5];
//        func_hex_to_str((char *)sum_check, data_temp); 
//        if(data_temp < 16) 												// func_hex_to_str函数转换0x0F的结果是0xF  因此需要移位
//	{
//		 
//                sum_check[3] = sum_check[2];
//		sum_check[2] = 0x30;
//	}
//	//PRINTF("%c,%c,%c,%c\r\n",verify_data[2],verify_data[3],receive_buffer[0],receive_buffer[1]);
//	if(verify_data[2] == receive_buffer[0] && verify_data[3] == receive_buffer[1])				// 和校验
//	{ 
//		bluetooth_ch9141_send_buff(&receive_buffer[3], 2); 										// 回复数据包ID，告知发送模块接收完毕
////		mutch_count++;
//		one_bag_num = (receive_buffer[3] - 0x30) * 16 + (receive_buffer[4]- 0x30);// 获取数据包ID
//                
//                
//                
//                //			for(uint8 i = 3; i < data_count; i ++) 							// 计算所有数据和
////	{
////          test +=  receive_buffer[i];
////                
////	}
//	//printf("%d\r\n",000000000000000000000000000);
//		strncpy((char *)receive_data, (char *)&receive_buffer[6], data_count - 6);	// 所有数据取出
//                        
//  			 //PRINTF("receive %d: %s,%c,%c,%d,%d\r\n", one_bag_num, (char *)receive_data,receive_buffer[0],receive_buffer[1],data_count,data_temp);	 // 将接收到的数据通过debug串口发送到上位机
//                       
//                            
//                            if(mutch_data[0]!=sum_check[2]||mutch_data[1]!=sum_check[3])
//                           {
////                              mutch_data[1]=one_bag_num;
//                              location_x[mutch_count] = (receive_data[0]-48)*10+receive_data[2]-48;
//                              location_y[mutch_count] = (receive_data[4]-48)*10+receive_data[6]-48;
//                              type[mutch_count] = receive_data[8]-48;
//                              
//                                  //PRINTF("%d,%d,%d,%c,%c,%c,asdasdasd,%s\r\n",location_x[mutch_count],location_y[mutch_count],type[mutch_count],receive_data[1],receive_data[2],receive_data[3],receive_data);
//                                  mutch_count++;
//                              gpio_init(B9,GPO,0,GPIO_PIN_CONFIG);
//                              ppp=1;
//                            }
////                         else if(mutch_data[0]!=one_bag_num)
////                         {
////                            mutch_count = 0;                        
////                         }
//                         mutch_data[0]=sum_check[2];
//                         mutch_data[1]=sum_check[3];
//                       }
//		
//	memset(receive_buffer, 0, sizeof(receive_buffer));				// 清空缓冲区					
//	
//	one_bag_receive_flag = 0;										// 解析结束
//}
//
//void bluetooth_init()
//{
//    //set_wireless_type(BLUETOOTH_CH9141, &bluetooth_ch9141_uart_callback);
//
//    //fifo_init(&bluetooth_ch9141_fifo, FIFO_DATA_8BIT, bluetooth_ch9141_buffer, BLUETOOTH_CH9141_BUFFER_SIZE);
//    // 本函数使用的波特率为115200 为蓝牙转串口模块的默认波特率 如需其他波特率请使用上位机修改模块参数
//    gpio_init(D26, GPI, 1, GPIO_PIN_CONFIG);                   // 初始化流控引脚
//    uart_init(BLUETOOTH_CH9141_INDEX, BLUETOOTH_CH9141_BUAD_RATE, BLUETOOTH_CH9141_RX_PIN, BLUETOOTH_CH9141_TX_PIN);
//    uart_rx_irq(BLUETOOTH_CH9141_INDEX,1); 
//    NVIC_SetPriority(LPUART8_IRQn,2);
//}