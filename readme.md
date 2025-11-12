# 9.12

# 板载外设（bsp_peripheral）

## 串口用途

### 底盘板

- 调试
- 裁判系统

### 云台板


- 调试 
- 遥控器
- 图传
- minipc

# 移植说明

***以下内容亲测有效，已在f407/f427/f405板上测试成功（f405因串口4有问题打印不了数据，故还未测试，因没有自定义按键和遥控器遥控的需求，故这两方面不需要移植）***

## 硬件

### LED

### KEY

### 调试用串口

看位号图和原理图接好线

### 遥控器

- 见新图传协议.md

### 裁判系统

需要自己做串口线（3pin-4pin  /  3pin-3pin）

***一定要注意线序***

## 软件

### cubemx

#### LED

看自己对应板子的原理图，找到端口，初始化为输出即可

#### KEY

看自己对应板子的原理图，找到端口，初始化为输入即可（***一定要设置为上拉或下拉***）

#### 调试用串口

- 随便选一个串口，添加dma的rx和tx，其中rx中选择circular
- 勾选该串口的全局中断
- 检查默认GPIO是否与原理图上的相同

#### 遥控器

- 使能串口（对应于板子上的dbus接口）（***具体是啥看原理图的dbus***），添加dma的rx，选择circular，priority最好选择very high
- 勾选该串口的全局中断
- 检查默认GPIO是否与原理图上的相同

***注意：遥控器通信波特率默认为100000***

#### 裁判系统

- 随便选一个串口，添加dma的rx和tx，其中rx中选择circular
- 勾选该串口的全局中断
- 检查默认GPIO是否与原理图上的相同

### clion

#### LED

- 将User/Framework/LED，User/MCUDriver/LED下的文件拷贝
- 修改cmake（include和file）
- LedC类的构造函数中三个参数分别是GPIOx  GPIO_PIN 和状态
- 要根据自己板子led灯是什么电平点亮修改ledio.cpp文件中的LEDIO_PortSetLedLevel()函数
- 一定要在main函数中初始化，即添加LEDIO_ConfigInit()

#### KEY

- 将User/Framework/KEY，User/MCUDriver/KEY下的文件拷贝
- 修改cmake（include和file）
- KeyC类中的构造函数中三个参数分别是GPIOx  GPIO_PIN 和触发时电平（根据实际情况修改）
- 一定要在main函数中初始化，即添加KEYIO_ConfigInit()

#### 调试用串口

- 将User/Framework/DEBUG中的文件拷贝
- 修改cmake（include和file）
- 将DEBUGC_UartInit()和DEBUGC_UartIrqHandler(UART_HandleTypeDef *huart)函数中有关串口huart的地方改成自己的串口
- 将debugc.cpp中的这三个函数DEBUGC_UartInit()  DEBUGC_UartIrqHandler(UART_HandleTypeDef *huart)  DEBUGC_UartIdleCallback(UART_HandleTypeDef *huart)在usart.h中声明
- 在stm32f4xx_it.c中的对应串口中断函数中调用DEBUGC_UartIrqHandler(UART_HandleTypeDef *huart) 
- 在main函数中加入DEBUGC_UartInit()  初始化对应串口dma中断

***说明：debugc.cpp中对应vofa调参部分，可参考debugc.h中的注释来设置vofa中的控件，也可自行修改***



#### 遥控器

- 将User/Framework/REMOTEC，User/MCUDriver/REMOTEC下的文件拷贝
- 修改cmake（include和file）
- 将REMOTEC_UartIrqHandler(void)和REMOTEIO_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)函数中的有关串口名称改成自己的
- remotec.cpp中REMOTEC_UartIrqHandler(void)函数里设定缓冲区的语句，其中的dma通道要改成自己的串口的rx通道

````c++
//设定缓冲区0
DMA2_Stream2->CR &= ~(DMA_SxCR_CT);
````

- 在usart.h中声明void REMOTEC_UartIrqHandler();
- 在usart.h中添加以下语句

```c
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
```

usart1换成自己所用串口

- 在stm32f4xx_it.c中的对应串口中断函数中调用REMOTEC_UartIrqHandler();
- 在主函数中调用REMOTEC_Init();

#### 裁判系统

- 将User/Framework/REFREE下的文件拷贝
- 修改cmake（include和file）
- 在stm32f4xx_it.c中的串口相关中断函数中添加以下语句

```c
//在串口rx的dma IRQHandler函数中
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */
  Judge_UseDMA_IRQHandler_RX();
  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}
```

```c
//在串口tx的dma IRQHandler函数中
void DMA2_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream7_IRQn 0 */
    Judge_UseDMA_IRQHandler_TX();
  /* USER CODE END DMA2_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_tx);
  /* USER CODE BEGIN DMA2_Stream7_IRQn 1 */

  /* USER CODE END DMA2_Stream7_IRQn 1 */
}
```

```c
//在串口全局中断函数中
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
    Judge_UseUART_IRQHandler();
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}
```

- 在usart.h中添加以下语句

```c
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
```

usart6换成自己的串口

- 在主函数中调用如下语句

```c++
Judge_UART_DMA_SET(huart6, hdma_usart6_rx, hdma_usart6_tx);
Judge_Init();
```

串口换成对应串口

#### c板陀螺仪

- 遇到__packed直接无脑删除，无影响（千万别乱添加头文件）
- 注意主函数中初始化顺序，dma的初始化一定要放在spi前面
- 注意stm32f4xx_it.c中的此函数，自己定义的函数一定要放在他默认生成的函数前面

```c
/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */
  SPI_RxCallBack();
  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */
  /* USER CODE END DMA2_Stream2_IRQn 1 */
}
```

### 测试效果

#### LED

#### KEY

#### 调试用串口

#### 遥控器

- 可通过以下语句测试是否移植成功

```c
usart_printf("%d, %d, %d, %d, %d, %d\r\n", rc_ctrl->rc.ch[0], rc_ctrl->rc.ch[1], rc_ctrl->rc.ch[2], rc_ctrl->rc.ch[3], rc_ctrl->rc.s[0], rc_ctrl->rc.s[1]);
```

- 更多遥控器接口请参考remotec.cpp中的遥控器协议解析函数sbus_to_rc()

#### 裁判系统

- 将板子与主控板连接（主控板未上电），串口会有如下提示


- 将主控板上电，可通过如下语句打印比赛剩余时间等变量（更多裁判系统接口函数请见judge.c文件的后半部分）

```c
usart_printf("%d,%d,%d,%d,%d,%d\r\n",Judge_Gametime()->stage_remain_time,Judge_GetRobotState()->robot_level,Judge_GetRobotState()->max_HP,Judge_GetRobotState()->remain_HP,Judge_GetRobotState()->mains_power_chassis_output,Judge_GetRobotState()->mains_power_gimbal_output);
```

- 未开始比赛时（主控板已上电），有如下信息（不应该全为0,全为0大概率是前面的步骤缺失）



- 开始比赛后，可以看到比赛剩余时间在减少。到此，裁判系统移植测试成功。



#### c板陀螺仪

- reset板子时，一定要保证陀螺仪（c板）处于静止状态。不然yaw轴会很偏

# 题外话

## git提交三部曲

- git add ...（若有其他文件夹中的文件需要一起上传，先cp到此本地仓库）
- git commit
- git push

***每日第一事：git pull***



# 24云台调试记录

## 初始化
- 定义兵种（gimbal.h）现在是omni
- 检查pitch、yaw反馈、算法; 目前是陀螺仪反馈+matlab生成前馈PID

      getMotorAngelAll_TypeDef YawMotorAllAngel = { 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, YAW_ANGLE,
                                                GYR_MODE, MATLAB};
      getMotorAngelAll_TypeDef PihMotorAllAngel = { 0, 0, 0, 0, 0,
                                                  0, 0, 0, 0, PIH_ANGLE,
                                                  GYR_MODE, NOMEL };
- 断电流，测限位（留出余量）：修改 `void Gimbal_CarInit(void)`（pitch轴、ecd反馈）；`Pih_GyrLimit`（pitch轴，gyr反馈）
- yaw轴与底盘：`void Gimbal_CarInit(void)`里修改 `ChassisYawTarget ;`
    - 11.15修改：由于云台反装，ChassisYawTarget从88改成 -94

## PID调试
- pid数值统一在：`Gimbal_CarChoose()`，需要根据兵种和算法修改对应pid，目前兵种omni，算法matlab
- for循环中，不要出现单独的常数下标，如0,1,2... （都统一用i作为下标）
- 
## 注意
- 关遥控器进保护模式
- 可以注释掉 `void Gimbal_SpeedC()`不输出电流，测角度

## to do list
- 研究摩擦轮电机
- 测试，调节弹道
- 测试拨弹轮卡弹保护
- 云台yaw轴pid细调



- 
