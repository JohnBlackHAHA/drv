#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <getopt.h>
#include <errno.h>
#include <pthread.h>//gcc编译要加 -pthread 参数或者 -lpthread
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/stat.h>
#include <time.h>
#include "/home/johnblack/zynq/ioctl.h"
static pthread_cond_t cond = PTHREAD_COND_INITIALIZER;//条件变量
static pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;//file读数据互斥锁
static const char* device = "/dev/axi_dev";
int rs232_fd;
static const char* rs232 = "/dev/ttyPS1";
static  TtmpData temdata;//数据内存缓冲区
typedef struct{
	unsigned int nFd;
}TZynqAxiHanddle;
TZynqAxiHanddle gHandle ={0};
static  int datapoint = 0;
static  int filepoint = 0;
static  int hardware_key = 0; //硬件开关
static  int fpga_ret = 0;//存状态返回值
static  int fpga_ret3=0;
static volatile int fpga_ret2 = 0;
static int files_count=0;
typedef struct {
    int head;
    int state;
    int data[18];
    int end;
}Buff;
typedef struct {
    int16_t data[2];
    int tmp;
}Data;
int send_data[18];
Data ReadAxiReg(int address)
{
    int ret = 0;
	TAxiRwReg nReg = {0};
	nReg.nRead = 1;
	int tmp;
	nReg.nRegAddr = address;
	ret = ioctl(gHandle.nFd,AXI_CMD_RW_REG,&nReg,sizeof(nReg));
	if(ret != 0)
	{
		perror("IOCTL");

	}
	tmp =nReg.nRegVal;
	if(nReg.nRegVal>=8232)
	{
		nReg.nRegVal=nReg.nRegVal>>13;
	}
	if(nReg.nRegVal == 0)
	{
		fpga_ret2=1;
	}
	Data data_raw;
	data_raw.data[0]=((nReg.nRegVal>>16)&0xFF)*0x100+(nReg.nRegVal>>24);
	data_raw.data[1]=(nReg.nRegVal&0xFF)*0x100+(nReg.nRegVal>>8);
	data_raw.tmp=tmp;
	return data_raw;
}
int WriteAxiReg(int data, int address)
{
    int length = 0;
    int count = 0;
	int ret = 0;
	TAxiRwReg nReg = {0};
	nReg.nRead = 0;
	nReg.nRegAddr = address;
	nReg.nRegVal = data;
	ret = ioctl(gHandle.nFd,AXI_CMD_RW_REG,&nReg,sizeof(nReg));
	if(ret != 0)
	{

		return 0x0002;
	}

	return 0x0001;
}
static int filestore()/*先在根目录下创建data文件夹，再在里面做文件夹创建和文件添加操作*/
{
	signal(SIGCHLD, SIG_DFL);
    mkdir("/data", S_IRWXU | S_IRWXG | S_IRWXO);
    system("mount /dev/mmcblk0p3 /data");
    while(1)
	{
        int irq_num=0;
        char dir_name[100] ="/data/file_";
        char str1[5],str2[5],str3[5],str4[5];
        char div[3] ="_";
        int put_int,get_int = 0;
        int fd = open(device, O_RDWR);
        Data timed= ReadAxiReg(0x1050);
   		int tmp = timed.tmp;
        char t1=(tmp>>24)&0xFF;
        char t2=(tmp>>16)&0xFF;
        char t3=(tmp>>8)&0xFF;
        char t4=tmp&0xFF;
        sprintf(str1,"%x",t1);
        sprintf(str2,"%x",t2);
        sprintf(str3,"%x",t3);
        sprintf(str4,"%x",t4);
        strcat(dir_name,str1);
        strcat(dir_name,div);
        strcat(dir_name,str2);
        strcat(dir_name,div);
        strcat(dir_name,str3);
   		strcat(dir_name,div);
        strcat(dir_name,str4);
        mkdir(dir_name,S_IRWXU | S_IRWXG | S_IRWXO);
   		char file_name[200] ={'\0'};
        char file_tmp[200]={'\0'};
        int file_num[4]={1,1,1,1};
        while (1)
        {
            int rc;
            pthread_mutex_lock(&mtx);
            rc=pthread_cond_wait(&cond, &mtx);
			if(rc == 0)
			{
                read(fd,&temdata,sizeof(temdata));
                if( temdata.length )
                {
                    int i =0;
                    int length = temdata.length;
                    int type = temdata.type;
                    FILE *fp = NULL;
                    char *name= "\0";
                    switch (type)
                    {
                        case 1:
                        name="/HDLC_";
                        break;
                        case 2:
                        name="/ETH_";
                        break;
                        case 3:
                        name="/1553_";
                        break;
                        default:
                        break;
                    }
                    char num[100]={'\0'};
                    sprintf(num,"%d",file_num[type]);
                    strcpy(file_name,file_tmp);
                    strcat(file_name,dir_name);
                    strcat(file_name,name);
                    strcat(file_name,num);
                    fp =fopen(file_name,"ab+");
                    if (fp<=0 )
                    {
						fclose(fp);
						fp =fopen(file_name,"ab+");
						perror("OPEN file FILEDEV");
                    }
                    if(fwrite(&temdata.data[0],sizeof(temdata.data[0]),length,fp)!=1)
                    {
			 			perror("FILE  write error!");

                    }
                    irq_num++;
                    if (irq_num == 100)//100次中断关闭当前文件
                    {
                        file_num[type]++;
                        irq_num=0;
                    }
					fclose(fp);
                    pthread_mutex_unlock(&mtx);
                }
				pthread_mutex_unlock(&mtx);
				if (fpga_ret3&&files_count==0)
				{
					files_count++;
					break;
				}
			}
        }
 	}
}
static void rs232rd()
{
	while(1)
	{
		usleep(5000);
		rs232_fd= open(rs232,O_RDWR|O_NOCTTY|O_NDELAY);
		fcntl(rs232_fd,F_SETFL,FNDELAY);
		if (rs232_fd <= 0)
        {
            perror("error1");
            return;
        }
        struct  termios rs232_opt;
        tcgetattr(rs232_fd,&rs232_opt);
        rs232_opt.c_cflag |= (CLOCAL | CREAD);
        rs232_opt.c_cflag &= ~PARODD;
        rs232_opt.c_cflag &= ~CSTOPB;
        rs232_opt.c_cflag &= ~CSIZE;
        rs232_opt.c_cflag |= CS8;
        rs232_opt.c_iflag &= ~(IXON | IXOFF | IXANY);
        rs232_opt.c_iflag &= ~(INLCR | IGNCR | ICRNL);
        rs232_opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        rs232_opt.c_oflag &= ~OPOST;
        rs232_opt.c_cc[VMIN] = 128;
        rs232_opt.c_cc[VTIME] = 1;
        cfsetispeed(&rs232_opt,B115200);  /*设置为115200Bps*/
        cfsetospeed(&rs232_opt,B115200);
        if(tcsetattr(rs232_fd, TCSANOW, &rs232_opt) != 0)
        {
            perror("could not set rs232 attributes\n");

        }
		int16_t tmp[] ={0xA55A,0x8207,0x1000,0x0000,0x0000};
		Data data_r=ReadAxiReg(0x4038);
		send_data[0]=data_r.tmp;
		tmp[3]=data_r.data[0];
		tmp[4]=data_r.data[1];
		write(rs232_fd,tmp,sizeof(tmp));
		tmp[2] =0x1400;
        data_r =ReadAxiReg(0x4034);
		send_data[1]=data_r.tmp;
        tmp[3]=data_r.data[0];
        tmp[4]=data_r.data[1];
        write(rs232_fd,tmp,sizeof(tmp));
		tmp[2] =0x1800;
        data_r =ReadAxiReg(0x4030);
		send_data[2]=data_r.tmp;
        tmp[3]=data_r.data[0];
        tmp[4]=data_r.data[1];
        write(rs232_fd,tmp,sizeof(tmp));
		tmp[2] =0x1C00;
        data_r =ReadAxiReg(0x4048);
		send_data[3]=data_r.tmp;
        tmp[3]=data_r.data[0];
        tmp[4]=data_r.data[1];
        write(rs232_fd,tmp,sizeof(tmp));
		tmp[2] =0x2000;
        data_r=ReadAxiReg(0x4044);
		send_data[4]=data_r.tmp;
        tmp[3]=data_r.data[0];
        tmp[4]=data_r.data[1];
        write(rs232_fd,tmp,sizeof(tmp));
		tmp[2] =0x2400;
        data_r =ReadAxiReg(0x4040);
		send_data[5]=data_r.tmp;
        tmp[3]=data_r.data[0];
        tmp[4]=data_r.data[1];
        write(rs232_fd,tmp,sizeof(tmp));
		tmp[2] =0x2800;
        data_r=ReadAxiReg(0x4078);
		send_data[6]=data_r.tmp;
        tmp[3]=data_r.data[0];
        tmp[4]=data_r.data[1];
        write(rs232_fd,tmp,sizeof(tmp));
		tmp[2] =0x2C00;
        data_r =ReadAxiReg(0x4074);
		send_data[7]=data_r.tmp;
        tmp[3]=data_r.data[0];
        tmp[4]=data_r.data[1];
        write(rs232_fd,tmp,sizeof(tmp));
		tmp[2] =0x3000;
        data_r =ReadAxiReg(0x4070);
		send_data[8]=data_r.tmp;
        tmp[3]=data_r.data[0];
        tmp[4]=data_r.data[1];
        write(rs232_fd,tmp,sizeof(tmp));
		tmp[2] =0x3400;
        data_r =ReadAxiReg(0x4088);
		send_data[9]=data_r.tmp;
        tmp[3]=data_r.data[0];
        tmp[4]=data_r.data[1];
        write(rs232_fd,tmp,sizeof(tmp));
		tmp[2] =0x3800;
        data_r =ReadAxiReg(0x4084);
		send_data[10]=data_r.tmp;
        tmp[3]=data_r.data[0];
        tmp[4]=data_r.data[1];
        write(rs232_fd,tmp,sizeof(tmp));
		tmp[2] =0x3C00;
        data_r =ReadAxiReg(0x4080);
		send_data[11]=data_r.tmp;
        tmp[3]=data_r.data[0];
        tmp[4]=data_r.data[1];
        write(rs232_fd,tmp,sizeof(tmp));
		tmp[2] =0x4000;
        data_r =ReadAxiReg(0x4058);
		send_data[12]=data_r.tmp;
        tmp[3]=data_r.data[0];
        tmp[4]=data_r.data[1];
        write(rs232_fd,tmp,sizeof(tmp));
		tmp[2] =0x4400;
        data_r =ReadAxiReg(0x4054);
		send_data[13]=data_r.tmp;
        tmp[3]=data_r.data[0];
        tmp[4]=data_r.data[1];
        write(rs232_fd,tmp,sizeof(tmp));
		tmp[2] =0x4800;
        data_r =ReadAxiReg(0x4050);
		send_data[14]=data_r.tmp;
        tmp[3]=data_r.data[0];
        tmp[4]=data_r.data[1];
        write(rs232_fd,tmp,sizeof(tmp));
		tmp[2] =0x4C00;
        data_r =ReadAxiReg(0x4068);
		send_data[15]=data_r.tmp;
        tmp[3]=data_r.data[0];
        tmp[4]=data_r.data[1];
        write(rs232_fd,tmp,sizeof(tmp));
		tmp[2] =0x5000;
        data_r =ReadAxiReg(0x4064);
		send_data[16]=data_r.tmp;
        tmp[3]=data_r.data[0];
        tmp[4]=data_r.data[1];
        write(rs232_fd,tmp,sizeof(tmp));
		tmp[2] =0x5400;
        data_r =ReadAxiReg(0x4060);
		send_data[17]=data_r.tmp;
        tmp[3]=data_r.data[0];
        tmp[4]=data_r.data[1];
        write(rs232_fd,tmp,sizeof(tmp));
		if(fpga_ret2==1&&fpga_ret==0x0003)
		{
			int16_t tmp1[]={0xA55A,0x820B,0x5800,0xFDCA,0xDDBE,0xECD2,0xA3B3};
			write(rs232_fd,tmp1,sizeof(tmp1));
			fpga_ret2=0;
		}
		else if(fpga_ret==0x0003)
		{
			int16_t tmp1[]={0xA55A,0x820B,0x5800,0xFDD5,0xA3B3,0xA4B9,0xF7D7};
			write(rs232_fd,tmp1,sizeof(tmp1));

		}
		tmp[2]=0x7800;
		system("df -h|grep mmcblk0p3|tr -d %|awk '{print $5}' > /data/sd");
		int fp = open("/data/sd", O_RDWR);
		int16_t ret;
		read(fp,&ret,sizeof(int16_t));
		int sd =atoi((char *)&ret);
        int16_t sd_r=(sd&0xFF)*0x100+(sd>>8);
		tmp[4]=sd_r;
        write(rs232_fd,tmp,sizeof(tmp));
		close(rs232_fd);
		close(fp);
		pthread_cond_signal(&cond);
	}
}
static void dosocket()
{
    int server_sockfd;//服务器端套接字
    int len;
    int i;
    int ret;//数据状态返回值
    struct sockaddr_in my_addr;   //服务器网络地址结构体
    struct sockaddr_in remote_addr; //客户端网络地址结构体
    int sin_size;
    Buff  buf={0};  //数据传送的缓冲区
    Buff * buf_p =&buf;
    memset(&my_addr,0,sizeof(my_addr)); //数据初始化--清零
    my_addr.sin_family=AF_INET; //设置为IP通信
    my_addr.sin_addr.s_addr=INADDR_ANY;//服务器IP地址
    my_addr.sin_port=htons(12000); //服务器端口号
    fd_set fds;
    struct timeval timeout={3,0};//三秒轮询
    server_sockfd= socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
    bind(server_sockfd,(struct sockaddr *)&my_addr,sizeof(struct sockaddr));
    listen(server_sockfd,5);
    sin_size=sizeof(struct sockaddr_in);
    //等待客户端连接请求到达
    while(1)
   	{
        int client_sockfd;
        if((client_sockfd=accept(server_sockfd,(struct sockaddr *)&remote_addr,&sin_size))== -1)
        {
            perror("SOCKET accept");
            return;
        }
        printf("accept client %s\n",inet_ntoa(remote_addr.sin_addr));
        while(1)//接受数据处理循环
        {
            FD_ZERO(&fds);
            FD_SET(client_sockfd,&fds);
            int maxfdp=client_sockfd+1;
            switch(select(maxfdp,&fds,&fds,NULL,&timeout))
            {
                case -1:break;
                case 0: break;
                default:
                if(FD_ISSET(client_sockfd,&fds))
                {
                    len=recv(client_sockfd,buf_p,sizeof(buf),0);
                    printf("len:%04x\n",len);
                    if(len<=0)
                        break;
                    if(buf.head!=0x5555||buf.end!=0x5A5A )
                    {
                        break;
                    }
                    printf("state:%04x\n",buf.state);
                    switch(buf.state)
                    {
                        case 0x0001:
                        buf.state= WriteAxiReg(1,0x100c);
                        break;
                        case 0x0002:
                        buf.state= WriteAxiReg(0,0x100c);
                        break;
                        case 0x0003:
                        buf.state = 1;
                        if(fpga_ret==0x0003)
                            buf.data[0]=1;
                        else
                            buf.data[0]=0;
                        break;
                        case 0x0004:
                        buf.state = 1;
                        for(i=0;i<17;i++)
                            buf.data[i]=send_data[i];
                        break;
                        default:
                        break;
                    }
                    if(send(client_sockfd,buf_p,len,0)<0)
                    {
                        perror("send");
                        break;
                    }
                }
                break;
            }
            if(len<=0)
          		break;
        }
        close(client_sockfd);
 	}
}
static void gpio_irq()
{
    signal(SIGCHLD, SIG_DFL);
    system("echo 36 > /sys/class/gpio/export");//假设是36号gpio口
    system("echo 37 > /sys/class/gpio/export");//假设是37号gpio口
    char ret2_pre='0',ret1_pre='0';
    while(1)
    {
        usleep(2000);
        int fp1 = open("/sys/class/gpio/gpio36/value", O_RDWR);
		int fp2 = open("/sys/class/gpio/gpio37/value", O_RDWR);
		if (fp1<=0||fp2<=0)
		{
			perror("open");
		}
		char ret1='\0',ret2='\0';
		read(fp1,&ret1,sizeof(char));
		read(fp2,&ret2,sizeof(char));
        if (ret1!=ret1_pre)
		{
                	WriteAxiReg(ret1-48,0x100c);
			if(ret1==48)
			{
				fpga_ret3=0;
				fpga_ret=0x0002;
			}
            else
            {
				fpga_ret=0x0003;
				files_count=0;
				fpga_ret3=1;
			}
		}
		if (ret2!=ret2_pre)
		WriteAxiReg(ret2-48,0x1008);
		ret1_pre=ret1;
		ret2_pre=ret2;
		close(fp1);
		close(fp2);
    }
}

int main(int argc, char *argv[])
{
	int count =0 ;
	int fd,data,address;
	fd = open(device, O_RDWR);
	gHandle.nFd =fd;
	pthread_t t1,t2,t3,t4;
	pthread_create(&t1,NULL,(void *)filestore,NULL);
	pthread_create(&t2,NULL,(void *)gpio_irq,NULL);
	pthread_create(&t3,NULL,(void *)rs232rd,NULL);
	pthread_create(&t4,NULL,(void *)dosocket,NULL);
	pthread_join(t1,NULL);
	pthread_join(t2,NULL);
	pthread_join(t3,NULL);
	pthread_join(t4,NULL);
	close(fd);
	close(rs232_fd);
	return 0;
}
