import network
import socket
from time import sleep
import machine
from machine import Pin, I2C
import time


#ssid = 'STARLINK v2'
ssid = 'get of my LAN'
password = 'P@$$w0rD'


def light_onboard_led():
    led = machine.Pin('LED', machine.Pin.OUT)
    led.on();
    time.sleep(0.1)
    led.off()



#Define I2C bus
i2c = I2C(0, sda=machine.Pin(0), scl=machine.Pin(1))

#Device address on the I2C bus
MPU6050_ADDR = 0x68

#PWR_MGMT_1 memory address
MPU6050_PWR_MGMT_1 = 0x6B

#Accelerometer and Gyroscope's high and low register for each axis
MPU6050_ACCEL_XOUT_H = 0x3B
MPU6050_ACCEL_XOUT_L = 0x3C
MPU6050_ACCEL_YOUT_H = 0x3D
MPU6050_ACCEL_YOUT_L = 0x3E
MPU6050_ACCEL_ZOUT_H = 0x3F
MPU6050_ACCEL_ZOUT_L = 0x40
MPU6050_GYRO_XOUT_H = 0x43
MPU6050_GYRO_XOUT_L = 0x44
MPU6050_GYRO_YOUT_H = 0x45
MPU6050_GYRO_YOUT_L = 0x46
MPU6050_GYRO_ZOUT_H = 0x47
MPU6050_GYRO_ZOUT_L = 0x48

#Accelerometer's LSB/g (least significant bits per gravitational force) sensitivity
MPU6050_LSBG = 16384.0

#Gyroscope's LSB/g sensitivity
MPU6050_LSBDS = 131.0


#Set all bits in the PWR_MGMT_1 register to 0
def mpu6050_init(i2c):
    i2c.writeto_mem(MPU6050_ADDR, MPU6050_PWR_MGMT_1, bytes([0]))


def combine_register_values(h, l):
    if not h[0] & 0x80:
        return h[0] << 8 | l[0]
    return -((h[0] ^ 255) << 8) |  (l[0] ^ 255) + 1


#Get Accelerometer values
def mpu6050_get_accel(i2c):
    accel_x_h = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, 1)
    accel_x_l = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_ACCEL_XOUT_L, 1)
    accel_y_h = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_ACCEL_YOUT_H, 1)
    accel_y_l = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_ACCEL_YOUT_L, 1)
    accel_z_h = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_ACCEL_ZOUT_H, 1)
    accel_z_l = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_ACCEL_ZOUT_L, 1)

    return [combine_register_values(accel_x_h, accel_x_l) / MPU6050_LSBG,
            combine_register_values(accel_y_h, accel_y_l) / MPU6050_LSBG,
            combine_register_values(accel_z_h, accel_z_l) / MPU6050_LSBG]


#Get Gyroscope values
def mpu6050_get_gyro(i2c):
    gyro_x_h = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_GYRO_XOUT_H, 1)
    gyro_x_l = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_GYRO_XOUT_L, 1)
    gyro_y_h = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_GYRO_YOUT_H, 1)
    gyro_y_l = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_GYRO_YOUT_L, 1)
    gyro_z_h = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_GYRO_ZOUT_H, 1)
    gyro_z_l = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_GYRO_ZOUT_L, 1)

    return [combine_register_values(gyro_x_h, gyro_x_l) / MPU6050_LSBDS,
            combine_register_values(gyro_y_h, gyro_y_l) / MPU6050_LSBDS,
            combine_register_values(gyro_z_h, gyro_z_l) / MPU6050_LSBDS]



def connect():
    #Connect to WLAN
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(ssid, password)
    while wlan.isconnected() == False:
        print('Waiting for connection...')
        sleep(1)
    ip = wlan.ifconfig()[0]
    print(f'Connected on {ip}')
    light_onboard_led()
    return ip

def open_socket(ip):
    # Open a socket
    address = (ip, 80)
    connection = socket.socket()
    connection.bind(address)
    connection.listen(1)
    return connection

def webpage(accelerometer_values, gyrometer_values):
    #Template HTML
    html = f"""


            <!DOCTYPE html>
            <html>

            <meta http-equiv="refresh" content="0.1">

            <head>
              <title>ROS2 Robot Control</title>
            </head>
            <center>
              <b>


              <body>
                <table>
                  <tr>
                    <td>
                      <p>Accelerometer:</p>
                    </td>
                    <td>
                      <p><span id="accelerometerValues">{accelerometer_values}</span></p>
                    </td>
                  </tr>
                </table>

                <table>
                  <tr>
                    <td>
                      <p>Gyroscope:</p>
                    </td>
                    <td>
                      <p><span id="gyrometerValues">{gyrometer_values}</span></p>
                    </td>
                  </tr>
                </table>
              </body>
            </html>


            """
    return str(html)

def serve(connection):
    i2c = I2C(0, sda=machine.Pin(0), scl=machine.Pin(1))
    mpu6050_init(i2c)
    #Start web server
    while True:

        print("Accelerometer:\t", mpu6050_get_accel(i2c), "g") #Print Accelerometer values (X,Y,Z)
        print("Gyroscope:\t", mpu6050_get_gyro(i2c), "°/s") #Print Gyroscope values (X,Y,Z)

        client = connection.accept()[0]
        request = client.recv(1024)
        request = str(request)
        try:
            request = request.split()[1]
        except IndexError:
            pass

        html = webpage(accelerometer_values=mpu6050_get_accel(i2c), gyrometer_values=mpu6050_get_gyro(i2c))
        client.send(html)
        client.close()

try:
    ip = connect()
    connection = open_socket(ip)
    serve(connection)
except KeyboardInterrupt:
    machine.reset()


