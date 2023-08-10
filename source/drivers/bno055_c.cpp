#include "bno055_c.hpp"

namespace drivers
{

    BNO055::BNO055(
            uint32_t            f_period, 
            PinName             SDA, 
            PinName             SCL,
            UnbufferedSerial&   f_serial
            ) 
        : _i2c(SDA,SCL)
        , utils::CTask(f_period)
        , m_serial(f_serial)
        , m_isActive(true)
        {
            //Set I2C fast and bring reset line high
            _i2c.frequency(400000);
            address = BNOAddress;
            accel_scale = 0.001f;
            rate_scale = 1.0f/16.0f;
            angle_scale = 1.0f/16.0f;
            temp_scale = 1;
            BNO055::reset();
        }

    /** @brief  CSerialMonitor class destructor
     */
    BNO055::~BNO055()
    {
    };
        
    void BNO055::reset()
    {
    //Perform a power-on-reset
        BNO055::readchar(BNO055_SYS_TRIGGER_ADDR);
        rx = rx | 0x20;
        BNO055::writechar(BNO055_SYS_TRIGGER_ADDR,rx);
    //Wait for the system to come back up again (datasheet says 650ms)
        ThisThread::sleep_for(675);
    }
        
    bool BNO055::check()
    {
    //Check we have communication link with the chip
        BNO055::readchar(BNO055_CHIP_ID_ADDR);
        if (rx != 0xA0) return false;
    //Grab the chip ID and software versions
        tx[0] = BNO055_CHIP_ID_ADDR;
        _i2c.write(address,tx,1,true);  
        _i2c.read(address+1,rawdata,7,false); 
        ID.id = rawdata[0];
        ID.accel = rawdata[1];
        ID.mag = rawdata[2];
        ID.gyro = rawdata[3];
        ID.sw[0] = rawdata[4];
        ID.sw[1] = rawdata[5];
        ID.bootload = rawdata[6];
        BNO055::setpage(1);
        tx[0] = BNO055_UNIQUE_ID_ADDR;
        _i2c.write(address,tx,1,true);  
        _i2c.read(address+1,ID.serial,16,false); 
        BNO055::setpage(0);
        return true;
        }
        
    void BNO055::SetExternalCrystal(bool yn)
    {
    // Read the current status from the device
        BNO055::readchar(BNO055_SYS_TRIGGER_ADDR); 
        if (yn) rx = rx | 0x80;
        else rx = rx & 0x7F;
        BNO055::writechar(BNO055_SYS_TRIGGER_ADDR,rx); 
    }

    void BNO055::set_accel_units(char units)
    {
        BNO055::readchar(BNO055_UNIT_SEL_ADDR);
        if(units == MPERSPERS){
            rx = rx & 0xFE;
            accel_scale = 0.01f;
            }
        else {
            rx = rx | units;
            accel_scale = 0.001f;
            }
        BNO055::writechar(BNO055_UNIT_SEL_ADDR,rx);
    }

    void BNO055::set_anglerate_units(char units)
    {
        BNO055::readchar(BNO055_UNIT_SEL_ADDR);
        if (units == DEG_PER_SEC){
            rx = rx & 0xFD;
            rate_scale = 1.0f/16.0f;
            }
        else {
            rx = rx | units;
            rate_scale = 1.0f/900.0f;
            }
        BNO055::writechar(BNO055_UNIT_SEL_ADDR,rx);
    }    

    void BNO055::set_angle_units(char units)
    {
        BNO055::readchar(BNO055_UNIT_SEL_ADDR);
        if (units == DEGREES){
            rx = rx & 0xFB;
            angle_scale = 1.0f/16.0f;
            }
        else {
            rx = rx | units;
            rate_scale = 1.0f/900.0f;
            }
        BNO055::writechar(BNO055_UNIT_SEL_ADDR,rx);
    }    

    void BNO055::set_temp_units(char units)
    {
        BNO055::readchar(BNO055_UNIT_SEL_ADDR);
        if (units == CENTIGRADE){
            rx = rx & 0xEF;
            temp_scale = 1;
            }
        else {
            rx = rx | units;
            temp_scale = 2;
            }
        BNO055::writechar(BNO055_UNIT_SEL_ADDR,rx);
    }    

    void BNO055::set_orientation(char units)
    {
        BNO055::readchar(BNO055_UNIT_SEL_ADDR);
        if (units == WINDOWS) rx = rx &0x7F;
        else rx = rx | units;
        BNO055::writechar(BNO055_UNIT_SEL_ADDR,rx);
    }        

    void BNO055::setmode(char omode)
    {
        BNO055::writechar(BNO055_OPR_MODE_ADDR,omode);
        op_mode = omode;
    }

    void BNO055::setpowermode(char pmode)
    {
        BNO055::writechar(BNO055_PWR_MODE_ADDR,pmode);
        pwr_mode = pmode;
    }

    void BNO055::get_accel(void)
    {
        tx[0] = BNO055_ACCEL_DATA_X_LSB_ADDR;
        _i2c.write(address,tx,1,true);  
        _i2c.read(address+1,rawdata,6,0); 
        accel.rawx = (rawdata[1] << 8 | rawdata[0]);
        accel.rawy = (rawdata[3] << 8 | rawdata[2]);
        accel.rawz = (rawdata[5] << 8 | rawdata[4]);
        accel.x = float(accel.rawx)*accel_scale;
        accel.y = float(accel.rawy)*accel_scale;
        accel.z = float(accel.rawz)*accel_scale;
    }
        
    void BNO055::get_gyro(void)
    {
        tx[0] = BNO055_GYRO_DATA_X_LSB_ADDR;
        _i2c.write(address,tx,1,true);  
        _i2c.read(address+1,rawdata,6,0); 
        gyro.rawx = (rawdata[1] << 8 | rawdata[0]);
        gyro.rawy = (rawdata[3] << 8 | rawdata[2]);
        gyro.rawz = (rawdata[5] << 8 | rawdata[4]);
        gyro.x = float(gyro.rawx)*rate_scale;
        gyro.y = float(gyro.rawy)*rate_scale;
        gyro.z = float(gyro.rawz)*rate_scale;
    }

    void BNO055::get_mag(void)
    {
        tx[0] = BNO055_MAG_DATA_X_LSB_ADDR;
        _i2c.write(address,tx,1,true);  
        _i2c.read(address+1,rawdata,6,0); 
        mag.rawx = (rawdata[1] << 8 | rawdata[0]);
        mag.rawy = (rawdata[3] << 8 | rawdata[2]);
        mag.rawz = (rawdata[5] << 8 | rawdata[4]);
        mag.x = float(mag.rawx);
        mag.y = float(mag.rawy);
        mag.z = float(mag.rawz);
    }

    void BNO055::get_lia(void)
    {
        tx[0] = BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR;
        _i2c.write(address,tx,1,true);  
        _i2c.read(address+1,rawdata,6,0); 
        lia.rawx = (rawdata[1] << 8 | rawdata[0]);
        lia.rawy = (rawdata[3] << 8 | rawdata[2]);
        lia.rawz = (rawdata[5] << 8 | rawdata[4]);
        lia.x = float(lia.rawx)*accel_scale;
        lia.y = float(lia.rawy)*accel_scale;
        lia.z = float(lia.rawz)*accel_scale;
    }

    void BNO055::get_grv(void)
    {
        tx[0] = BNO055_GRAVITY_DATA_X_LSB_ADDR;
        _i2c.write(address,tx,1,true);  
        _i2c.read(address+1,rawdata,6,0); 
        gravity.rawx = (rawdata[1] << 8 | rawdata[0]);
        gravity.rawy = (rawdata[3] << 8 | rawdata[2]);
        gravity.rawz = (rawdata[5] << 8 | rawdata[4]);
        gravity.x = float(gravity.rawx)*accel_scale;
        gravity.y = float(gravity.rawy)*accel_scale;
        gravity.z = float(gravity.rawz)*accel_scale;
    }

    void BNO055::get_quat(void)
    {
        tx[0] = BNO055_QUATERNION_DATA_W_LSB_ADDR;
        _i2c.write(address,tx,1,true);  
        _i2c.read(address+1,rawdata,8,0); 
        quat.raww = (rawdata[1] << 8 | rawdata[0]);
        quat.rawx = (rawdata[3] << 8 | rawdata[2]);
        quat.rawy = (rawdata[5] << 8 | rawdata[4]);
        quat.rawz = (rawdata[7] << 8 | rawdata[6]);
        quat.w = float(quat.raww)/16384.0f;
        quat.x = float(quat.rawx)/16384.0f;
        quat.y = float(quat.rawy)/16384.0f;
        quat.z = float(quat.rawz)/16384.0f;
    }

    void BNO055::get_angles(void)
    {
        tx[0] = BNO055_EULER_H_LSB_ADDR;
        _i2c.write(address,tx,1,true);  
        _i2c.read(address+1,rawdata,6,0); 
        euler.rawyaw = (rawdata[1] << 8 | rawdata[0]);
        euler.rawroll = (rawdata[3] << 8 | rawdata[2]);
        euler.rawpitch = (rawdata[5] << 8 | rawdata[4]);
        euler.yaw = float(euler.rawyaw)*angle_scale;
        euler.roll = float(euler.rawroll)*angle_scale;
        euler.pitch = float(euler.rawpitch)*angle_scale;
    }


    void BNO055::get_temp(void)
    {
        BNO055::readchar(BNO055_TEMP_ADDR);
        temperature = rx / temp_scale;
    }

    void BNO055::get_calib(void)
    {
        BNO055::readchar(BNO055_CALIB_STAT_ADDR);
        calib = rx;
    }

    void BNO055::read_calibration_data(void)
    {
        char tempmode = op_mode;
        setmode(OPERATION_MODE_CONFIG);
        ThisThread::sleep_for(20);
        tx[0] = ACCEL_OFFSET_X_LSB_ADDR;
        _i2c.write(address,tx,1,true);  
        _i2c.read(address,calibration,22,false); 
        setmode(tempmode);
        ThisThread::sleep_for(10);
    }

    void BNO055::write_calibration_data(void)
    {
        char tempmode = op_mode;
        setmode(OPERATION_MODE_CONFIG);
        ThisThread::sleep_for(20);
        tx[0] = ACCEL_OFFSET_X_LSB_ADDR;
        _i2c.write(address,tx,1,true);  
        _i2c.write(address,calibration,22,false); 
        setmode(tempmode);
        ThisThread::sleep_for(10);
    }

    void BNO055::set_mapping(char orient)
    {
        switch (orient)
        {
            case 0: 
                BNO055::writechar(BNO055_AXIS_MAP_CONFIG_ADDR,0x21);
                BNO055::writechar(BNO055_AXIS_MAP_SIGN_ADDR,0x04);
                break;
            case 1:
                BNO055::writechar(BNO055_AXIS_MAP_CONFIG_ADDR,0x24);
                BNO055::writechar(BNO055_AXIS_MAP_SIGN_ADDR,0x00);
                break;
            case 2:
                BNO055::writechar(BNO055_AXIS_MAP_CONFIG_ADDR,0x24);
                BNO055::writechar(BNO055_AXIS_MAP_SIGN_ADDR,0x00);
                break;
            case 3:
                BNO055::writechar(BNO055_AXIS_MAP_CONFIG_ADDR,0x21);
                BNO055::writechar(BNO055_AXIS_MAP_SIGN_ADDR,0x02);
                break;
            case 4:
                BNO055::writechar(BNO055_AXIS_MAP_CONFIG_ADDR,0x24);
                BNO055::writechar(BNO055_AXIS_MAP_SIGN_ADDR,0x03);
                break;
            case 5:
                BNO055::writechar(BNO055_AXIS_MAP_CONFIG_ADDR,0x21);
                BNO055::writechar(BNO055_AXIS_MAP_SIGN_ADDR,0x01);
                break;
            case 6:
                BNO055::writechar(BNO055_AXIS_MAP_CONFIG_ADDR,0x21);
                BNO055::writechar(BNO055_AXIS_MAP_SIGN_ADDR,0x07);
                break;
            case 7:
                BNO055::writechar(BNO055_AXIS_MAP_CONFIG_ADDR,0x24);
                BNO055::writechar(BNO055_AXIS_MAP_SIGN_ADDR,0x05);
                break;
            default:
                BNO055::writechar(BNO055_AXIS_MAP_CONFIG_ADDR,0x24);
                BNO055::writechar(BNO055_AXIS_MAP_SIGN_ADDR,0x00);
        }
    }

    void BNO055::readchar(char location){
        tx[0] = location;
        _i2c.write(address,tx,1,true);
        _i2c.read(address,&rx,1,false);
    }

    void BNO055::writechar(char location, char value){
        tx[0] = location;
        tx[1] = value;
        _i2c.write(address,tx,2);
    }

    void BNO055::setpage(char value){
        writechar(BNO055_PAGE_ID_ADDR,value);
    }

    /** \brief  Serial callback method to activate or deactivate the publisher. 
     * When the received integer value is bigger or equal to 1, then the publisher become 
     * active and send messages, otherwise is deactivated. 
     *
     * @param a                   input received string
     * @param b                   output reponse message
     * 
     */
    void BNO055::ImuPublisherCommand(char const * a, char * b) {
        int l_isActivate=0;
        uint32_t l_res = sscanf(a,"%d",&l_isActivate);
        if(l_res==1){
            m_isActive=(l_isActivate>=1);
            sprintf(b,"ack;;");
        }else{
            sprintf(b,"sintax error;;");
        }
    }

    /** \brief  Periodically applied method to check the ADC value
     * 
     */
    void BNO055::_run()
    {
        if(!m_isActive) return;
        bool a = BNO055::check();
        if (a)
        {
            m_serial.write("on",2);
        }
        else
        {
            m_serial.write("off",3);
        }
        // m_serial.printf("@6:%.2f;;\r\n", l_rps);
    }
}