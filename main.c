#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "rtthread.h"
#include "dev_sign_api.h"
#include "mqtt_api.h"
#include <board.h>
#include <rtdevice.h>
#include "cJSON.h"

/*定义pin引脚 */设备
#define K1 GET_PIN(C,13)
#define lock GET_PIN(A,10)
#define sound GET_PIN(B,0)
/*声明中断*/
void irq_k1(void *args);
void irq_sound(void *args);

/* 定义串口设备句柄 */
//rt_device_t uart3;
rt_device_t uart5;
rt_device_t serial;//指纹设备句柄

uint8_t flag=0;//指纹标识符
/*握手数组*/
uint8_t PS_GetChipEcho[12]={0xef,0x01,0xff,0xff,0xff,0xff,0x01,0x00,0x03,0x35,0x00,0x39};
/*自动注册数组*/
uint8_t PS_AutoEnroll[17]={0xef,0x01,0xff,0xff,0xff,0xff,0x01,0x00,0x08,0x31,0x00,0x01,0x02,0x00,0x2c,0x00,0x69};
/* 自动验证指纹数组 */
uint8_t PS_AutoIdentify[17]={0xef,0x01,0xff,0xff,0xff,0xff,0x01,0x00,0x08,0x32,0x02,0x00,0x01,0x00,0x05,0x00,0x43};
/* 接收数组 */
uint8_t msg[18];
uint8_t screen[8];
struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;  /* 初始化配置参数 */

char *topic = "/sys/iucaFXn17zL/lock/thing/service/property/set";//订阅主题
char *topic1 = "/sys/iucaFXn17zL/lock/thing/event/property/post";//发布主题
void *pclient = NULL;

/*定义一个结构体，表示门锁状态 */
struct msg{
    int LockSwitch;
    int LockState;
};

int Lock,a;

rt_mq_t mq_g_u;//消息队列
rt_sem_t sem_i_g;//信号量
rt_sem_t uart3_rcv;//串口3接收信号量
rt_sem_t uart4_rcv;//串口4接收信号量
rt_sem_t sem_uart;//指纹串口接收信号量

int len_cmd = 0;

extern int HAL_Snprintf(char *str, const int len, const char *fmt, ...);//格式化输出字符串，并将结果写入到指定的缓冲区

/* 下发数据解析 */
void msg_recv_parse(char *json_string)
{
    struct msg msg_change;
    msg_change.LockState = 0;

    /* 创建解析节点 */
    cJSON *msg_json = NULL;
    /* 解析数据 */
    msg_json = cJSON_Parse(json_string);
    cJSON *msg_parmas = cJSON_GetObjectItem(msg_json, "params");
     if(cJSON_HasObjectItem(msg_parmas, "LockState"))
        {
            Lock = cJSON_GetObjectItem(msg_parmas, "LockState")->valueint;
            if (Lock==0) {
                rt_pin_write(lock, 0);
                msg_change.LockState = 0;
            }
            if (Lock==1) {
                            rt_pin_write(lock, 1);
                            msg_change.LockState = 1;
                        }
            rt_mq_send(mq_g_u, &msg_change, sizeof(msg_change));//发送消息
            rt_kprintf("LockState: %d\n", Lock);
        }
    /* 删除节点，释放内存 */
    cJSON_Delete(msg_json);
}

static void message_arrive(void *pcontext, void *pclient, iotx_mqtt_event_msg_pt msg)
{
    iotx_mqtt_topic_info_t     *topic_info = (iotx_mqtt_topic_info_pt) msg->msg;

    switch (msg->event_type) {
        case IOTX_MQTT_EVENT_PUBLISH_RECEIVED:
            msg_recv_parse(topic_info->payload);
            break;
        default:
           break;
    }
}

void ali_init_entry(void *parameter)//阿里云初始化
{
    iotx_mqtt_param_t mqtt_params;  //构造客户端
    rt_thread_mdelay(20000);
    memset(&mqtt_params, 0x0, sizeof(mqtt_params));//内存初始化
    pclient = IOT_MQTT_Construct(&mqtt_params); //连接MQTT
    IOT_MQTT_Subscribe(pclient, topic, IOTX_MQTT_QOS0, message_arrive, NULL);//订阅主题
    rt_sem_release(sem_i_g);//释放信号量
}

void get_data_entry(void *parameter)//获取数据
{
    struct msg msg_get;//定义一个结构体
    msg_get.LockState = 0;

    rt_sem_take(sem_i_g, RT_WAITING_FOREVER);//获取信号量
    while(1)
    {
      /*读取门锁数据 */
        //a=rt_pin_read(lock);//判断门锁是否改变，改变则上传数据
        if (rt_pin_read(lock)!=PIN_LOW) {
            msg_get.LockState = 1;//灯亮，锁开
        }
       else {
            msg_get.LockState = 0;//灯灭，关
        }
        rt_mq_send(mq_g_u, &msg_get, sizeof(msg_get));//发送消息
        rt_thread_mdelay(1000);
//        if(a!=rt_pin_read(lock))
//        {
//        rt_mq_send(mq_g_u, &msg_get, sizeof(msg_get));//发送消息
//        }
//        else {
//               continue;
//        }
//        //rt_thread_mdelay(1000);
    }
}

void upload_entry(void *parameter)//上传数据
{
    struct msg msg_upload;//定义结构体
    //char *payload_fmt = "{\"params\":{\"temp\":%d,\"humi\":%d}}";
    char *payload_fmt = "{\"params\":{\"LockState\":%d}}";
    char *payload = NULL;
    int payload_len = strlen(payload_fmt)+4;
    while(1)
    {
        rt_mq_recv(mq_g_u, &msg_upload, sizeof(msg_upload), RT_WAITING_FOREVER);//接收消息
        payload = rt_malloc(payload_len);//开辟存储
        memset(payload, 0, payload_len);//复制数据到payload

       // HAL_Snprintf(payload, payload_len, payload_fmt, msg_upload.temp, msg_upload.humi);//格式化
        HAL_Snprintf(payload, payload_len, payload_fmt, msg_upload.LockState);
        //rt_kprintf("%s\n",payload);
        IOT_MQTT_Publish_Simple(0, topic1, IOTX_MQTT_QOS0, payload, strlen(payload));//发布消息

        IOT_MQTT_Yield(pclient, 200);

        rt_free(payload);//释放存储
    }
}

void pin_device_entry(void *parameter)
{
    //rt_kprintf("in pin_device_entry\n");
    rt_pin_mode(lock, PIN_MODE_OUTPUT);//LED1推挽输出
     rt_pin_mode(K1, PIN_MODE_INPUT_PULLDOWN);//K1下拉输入

     /*  绑定K1中断回调函数，上升沿触发中断 */
      rt_pin_attach_irq(K1, PIN_IRQ_MODE_RISING, irq_k1, RT_NULL);

      /*  使能引脚中断 */
      rt_pin_irq_enable(K1, PIN_IRQ_ENABLE);
}


rt_err_t uart3_recv_callback(rt_device_t dev, rt_size_t size)
{
    /* 串口接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
   // rt_kprintf("in uart3 callback\n");
      rt_sem_release(uart3_rcv);

      return RT_EOK;

}
void screen_data_rcv_entry(void *parameter)
{
    while (1)
        {
            rt_thread_mdelay(10);
            /* 从串口读取一个字节的数据，没有读取到则等待接收信号量 */
            while (rt_device_read(uart5, 0, screen, sizeof(screen)) != RT_NULL)
            {
                /* 阻塞等待接收信号量，等到信号量后再次读取数据 */
                rt_sem_take(uart3_rcv, RT_WAITING_FOREVER);
                rt_thread_mdelay(100);
               // rt_kprintf("screen[0]:%d\n",screen[0]);
                if(screen[0]==1)
                {
                    rt_pin_write(lock, 1);
                }
                if(screen[0]==2)
                {
                    rt_pin_write(lock, 0);
                }
            }
        }
}


void screen_entry(void *parameter)
{
    /* 查找串口设备 */
    uart5 = rt_device_find("uart5");

    /*  打开串口设备，中断接收 */
    rt_device_open(uart5, RT_DEVICE_FLAG_INT_RX);

    /*  设置接收回调函数 */
    rt_device_set_rx_indicate(uart5, uart3_recv_callback);
}

void voice_entry(void *parameter)
{

   rt_pin_mode(sound, PIN_MODE_INPUT_PULLDOWN);//K1下拉输入
   while(1)
   {
       if(rt_pin_read(sound)==PIN_HIGH)
           {
              // rt_kprintf("door open\n");
               rt_pin_write(lock, 1);
               rt_thread_mdelay(5000);
               rt_pin_write(lock, 0);
           }
//       if(rt_pin_read(sound)==PIN_LOW)
//       {
//           rt_pin_write(lock, 0);
//       }
       rt_thread_mdelay(100);
   }
    /*  绑定K1中断回调函数，上升沿触发中断 */
//   rt_pin_attach_irq(sound, PIN_IRQ_MODE_RISING, irq_sound, RT_NULL);
//      /* 使能引脚中断 */
//   rt_pin_irq_enable(sound, PIN_IRQ_ENABLE);

}

//void irq_sound(void *args)
//{
//    if(rt_pin_read(sound)==PIN_HIGH)
//    {
//       // rt_kprintf("door open\n");
//        rt_pin_write(lock, 1);
//        rt_thread_mdelay(5000);
//        rt_pin_write(lock, 0);
//    }
//}

/* 串口接收回调函数 */
rt_err_t uart_recv_callback(rt_device_t dev, rt_size_t size)
{
    //rt_kprintf("in uart4 callback!\n");
    rt_sem_release(sem_uart);
    return RT_EOK;
}

void t_uartinit_entry(void *parameter)
{
    rt_err_t ret = RT_EOK;
    serial = rt_device_find("uart4");
    if (!serial)
    {
        rt_kprintf("find uart4 failed!\n");
        return;
    }
    else {
        rt_kprintf("find uart4 ok!\n");
    }

   /* 修改串口配置参数 */
   config.baud_rate = BAUD_RATE_57600;        //修改波特率为 57600
   config.data_bits = DATA_BITS_8;           //数据位 8
   config.stop_bits = STOP_BITS_1;           //停止位 1
   config.bufsz     = 128;                   //修改缓冲区 buff size 为 128
   config.parity    = PARITY_NONE;           //无奇偶校验位

   /*控制串口设备。通过控制接口传入命令控制字，与控制参数 */
   rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);

    /* 以中断接收及轮询发送模式打开串口设备 */
    ret = rt_device_open(serial, RT_DEVICE_FLAG_INT_RX);
    if (ret != RT_EOK)
    {
       rt_kprintf("open device failed\r\n");
       return;
    }
    else {
        rt_kprintf("open device ok\r\n");
        //rt_device_write(serial, 0, PS_GetChipEcho, (sizeof(PS_GetChipEcho)));
        rt_device_write(serial, 0, PS_GetChipEcho, (sizeof(PS_GetChipEcho)));//发送握手信息
    }
    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(serial, uart_recv_callback);
}

void t_uartrec_entry(void *parameter)
{
    char ch;
    int i = 0;
    while (1)
    {
        /* 从串口读取一个字节的数据，没有读取到则等待接收信号量 */
        while (rt_device_read(serial, -1, &ch, 1) != 1)
        {
            /* 阻塞等待接收信号量，等到信号量后再次读取数据 */
            rt_sem_take(sem_uart, RT_WAITING_FOREVER);
        }
        /* 读取到的数据通过串口输出 */
       // rt_device_write(serial, 0, &ch, 1);
        msg[i++] = ch;
        if (i-1 == 8) {
            len_cmd = msg[i-1]+9;
        } //获取指令长度
        /* 接收完成，解析数据 */
        if (i==len_cmd) {
            i = 0;

            if(flag==1)
            {
                if(msg[11]==0x0a)
                {
                    rt_kprintf("PS_GetChipEcho_ok\n");
                }
            }
            if(flag==2)
            {
                if(msg[10]==0x06)
                {
                    if(msg[11]==0xf2)
                    {
                        rt_kprintf("PS_AutoEnroll_ok\n");
                    }
                }
            }
            if(flag==3)
            {
                flag=0;
                if(msg[9]==0x00)
                {
                    if(msg[10]==0x05)
                    {
                        rt_kprintf("PS_AutoIdentify_ok\n");
                        //rt_kprintf("door open\n");
                        rt_pin_write(lock, 1);
                        rt_thread_mdelay(5000);
                        rt_pin_write(lock, 0);

                    }
                }
            }
        }
    }
}

int main(void)
{

    /* 定义线程句柄 */
    rt_thread_t ali_init, get_data, upload, pin_device,screen,screen_data_rcv,voice,t_uartinit,t_uartrec;//初始化，获取数据，上传

    /* 创建消息队列 */
    mq_g_u = rt_mq_create("mqgu", 100, 10, RT_IPC_FLAG_FIFO);

    /* 创建信号量 */
    sem_i_g = rt_sem_create("semig", 0, RT_IPC_FLAG_FIFO);

    uart3_rcv = rt_sem_create("uart3rcv", 0, RT_IPC_FLAG_FIFO);

    uart4_rcv = rt_sem_create("uart4rcv", 0, RT_IPC_FLAG_FIFO);

    sem_uart = rt_sem_create("semuart", 0, RT_IPC_FLAG_FIFO);

    /* 创建线程 */
    ali_init = rt_thread_create("ali_init", ali_init_entry, RT_NULL, 4096, 6, 10);
    get_data = rt_thread_create("get_data", get_data_entry, RT_NULL, 4096, 8, 10);
    upload = rt_thread_create("upload", upload_entry, RT_NULL, 4096, 7, 10);
    pin_device =  rt_thread_create("pin_device", pin_device_entry, RT_NULL, 4096, 8, 10);
    screen =  rt_thread_create("screen", screen_entry, RT_NULL, 4096, 8, 10);
    screen_data_rcv =  rt_thread_create("screendatarcv", screen_data_rcv_entry, RT_NULL, 4096, 8, 10);
    voice =  rt_thread_create("voice", voice_entry, RT_NULL, 4096, 8, 10);
    t_uartinit = rt_thread_create("uartinit", t_uartinit_entry, RT_NULL, 1024, 10, 10);
    t_uartrec = rt_thread_create("uartrec", t_uartrec_entry, RT_NULL, 1024, 10, 10);


    /* 启动线程 */
    rt_thread_startup(t_uartinit);
    rt_thread_startup(t_uartrec);
    rt_thread_startup(ali_init);
    rt_thread_startup(get_data);
    rt_thread_startup(upload);
    rt_thread_startup(pin_device);
    rt_thread_startup(screen);
    rt_thread_startup(screen_data_rcv);
    rt_thread_startup(voice);


    return 0;
}
/* 中断回调函数 */
void irq_k1(void *args)
{
    flag++;
    if(flag==1)
    {
        rt_device_write(serial, 0, PS_GetChipEcho, sizeof(PS_GetChipEcho));//发送握手信息
        rt_kprintf("send1,ok\n");
    }
    if(flag==2)
    {
        rt_device_write(serial, 0, PS_AutoEnroll, sizeof(PS_AutoEnroll));//发送注册信息
        rt_kprintf("send2,ok\n");
    }
    if(flag==3)
    {
        rt_device_write(serial, 0, PS_AutoIdentify, sizeof(PS_AutoIdentify));//发送验证信息
        rt_kprintf("send3,ok\n");
    }
}

