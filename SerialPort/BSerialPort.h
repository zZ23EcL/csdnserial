/* 
 * File:   BSerialPort.h
 */

#ifndef BSERIALPORT_H
#define BSERIALPORT_H

class BSerialPort {
public:
    friend class BSerialPortManger;
    
    enum SerialType{
        Actual, Virtual,
    };
    
    BSerialPort();
    BSerialPort(const BSerialPort& orig);
    BSerialPort(char *filename, SerialType type = Actual);
    BSerialPort(const char *filename, SerialType type = Actual);
    
    virtual ~BSerialPort();
    
    /*
     * 参数: 串口设备文件路径
     * 功能：打开串口，默认设置串口设备参数：9600,N,1
     * 返回值：成功 true ; 失败 false
     */
    bool open(char *filename);
    /*
     * 参数： 串口波特率
     * 功能： 设置串口模特率，可以是下面的一个值
     * 921600, 460800, 230400, 115200, 57600, 38400, 19200, 9600, 4800, 2400, 1200, 300
     * 返回值： 成功 true ; 失败 false
     */
    bool set_Speed(int speed);
    /*
     * 参数： databits：数据位数
     *       stopbits：停止位
     *       parity：  校验位
     * 功能： 设置串口的数据位、停止位、校验位。
     * 返回值： 成功 true ; 失败 false
     */
    bool set_Parity(int databits, int stopbits, char parity);
    /*
     * 参数：dest：数据读取缓存数组
     *      bytes：要读取数据的字节数
     * 功能：从串口读取数据存储在dest中
     * 返回值：实际读取数据字节数，出错返回-1。
     */
    int  read(char *dest, int bytes);
    /*
     * 参数：buff：要写入的数据
     *      bytes：要写入数据的字节数
     * 功能：将buff中的数据写入串口
     * 返回值：实际写入数据字节数，出错返回-1。
     */
    int  write(char *buff, int bytes);
    /*关闭串口*/
    void close(void);
    
    bool operator==(BSerialPort &serial);
private:
    /*串口文件描述符*/
    int fd;
    /*如果是虚拟串口，从设备的文件描述符*/
    int sfd;
    void (*function)(BSerialPort *serial);
};
#endif /* BSERIALPORT_H */

