#include "periodics/ultrasonido.hpp"
#include "mbed.h"

#define TOT_MUESTRAS 1 //Sin promediar

// TODO: Add your code here
namespace periodics
{
    /**
     * @brief Class constructor ultrasonido
     *
     */
    CUltrasonido::CUltrasonido(
            std::chrono::milliseconds f_period,
            UnbufferedSerial &f_serial,
            drivers::ISpeedingCommand& f_speedingControl,
            mbed::DigitalOut &TrigPin,
            mbed::InterruptIn &EchoFront,
            mbed::InterruptIn &EchoBack,
            mbed::InterruptIn &EchoLeft,
            mbed::InterruptIn &EchoRight
        )
        : utils::CTask(f_period)
        , m_serial(f_serial)
        , m_trigger(TrigPin)
        , m_speedControl(f_speedingControl)
        , m_echoFront(EchoFront)
        , m_echoBack(EchoBack)
        , m_echoLeft(EchoLeft)
        , m_echoRight(EchoRight)
        , m_isActive(false)
        , n_muestras(0)
        , prom_muestrasFront(0)
        , prom_muestrasBack(0)
        , prom_muestrasLeft(0)
        , prom_muestrasRight(0)
        , l_distanceFront(0)
        , l_distanceBack(0)
        , l_distanceLeft(0)
        , l_distanceRight(0)
    {
        /* constructor behaviour */
        m_trigger = 0;
        m_objectDetected = false;
        m_echoFront.mode(PullDown);
        m_echoBack.mode(PullDown);
        m_echoLeft.mode(PullDown);
        m_echoRight.mode(PullDown);
    }

    /** @brief  CUltrasonido class destructor
     */
    CUltrasonido::~CUltrasonido()
    {
    }

    void CUltrasonido::serialCallbackULTRAcommand(char const * a, char * b) {
        uint8_t l_isActivate = 0;
        uint8_t l_res = sscanf(a, "%hhu", &l_isActivate);

        if (1 == l_res) {
            if (uint8_globalsV_value_of_kl == 15 || uint8_globalsV_value_of_kl == 30) {
                m_isActive = (l_isActivate >= 1);
                bool_globalsV_ultra_isActive = (l_isActivate >= 1);
                sprintf(b, "1");
            } else {
                sprintf(b, "kl 15/30 is required!!");
            }
        } else {
            sprintf(b, "syntax error");
        }
    }

    /* Run method */
    void CUltrasonido::_run()
    {
        /* Run method behaviour */
        if (!m_isActive) return;

        m_trigger = 1;
        ThisThread::sleep_for(chrono::microseconds(10).count() / 1000); // Convert microseconds to milliseconds
        m_trigger = 0;

        m_echoFront.rise(mbed::callback(this, &CUltrasonido::startTimerFront));
        m_echoFront.fall(mbed::callback(this, &CUltrasonido::stopTimerFront));

        m_echoBack.rise(mbed::callback(this, &CUltrasonido::startTimerBack));
        m_echoBack.fall(mbed::callback(this, &CUltrasonido::stopTimerBack));

        m_echoLeft.rise(mbed::callback(this, &CUltrasonido::startTimerLeft));
        m_echoLeft.fall(mbed::callback(this, &CUltrasonido::stopTimerLeft));

        m_echoRight.rise(mbed::callback(this, &CUltrasonido::startTimerRight));
        m_echoRight.fall(mbed::callback(this, &CUltrasonido::stopTimerRight));

        //--------------------------------------------------------------------------------------//
        //                                  Muestreo y promedio                                 //
        //--------------------------------------------------------------------------------------//


        if (n_muestras < TOT_MUESTRAS )
        {
            n_muestras++;
            prom_muestrasFront += l_distanceFront;
            prom_muestrasBack += l_distanceBack;
            prom_muestrasLeft += l_distanceLeft;
            prom_muestrasRight += l_distanceRight;
        }
        else
        {
            if( l_distanceFront <= 30 && l_distanceBack <= 30 )
            {
                if (m_objectDetected == false)
                {
                    m_speedControl.setBrake();
                    m_objectDetected = true;
                }
                else
                {
                    printf("@ultra:500;500;500;500;;\n\r");     //objetos detectados
                }
            }

            printf("@ultra:%d;%d;%d;%d;;\n\r"
                , l_distanceFront/TOT_MUESTRAS
                , l_distanceBack/TOT_MUESTRAS
                , l_distanceLeft/TOT_MUESTRAS
                , l_distanceRight/TOT_MUESTRAS
                );
            n_muestras = 0;
            prom_muestrasFront = 0;
            prom_muestrasBack = 0;
            prom_muestrasLeft = 0;
            prom_muestrasRight = 0;
        }
    }

    void CUltrasonido::startTimerFront() {
        m_timerFront.reset();
        m_timerFront.start();
    }

    void CUltrasonido::stopTimerFront() {
        m_timerFront.stop();
        int duration = m_timerFront.elapsed_time().count(); // Get elapsed time in microseconds
        l_distanceFront = duration / 58.0; // Convert time to distance
    }

    void CUltrasonido::startTimerBack() {
        m_timerBack.reset();
        m_timerBack.start();
    }

    void CUltrasonido::stopTimerBack() {
        m_timerBack.stop();
        int duration = m_timerBack.elapsed_time().count(); // Get elapsed time in microseconds
        l_distanceBack = duration / 58.0; // Convert time to distance
    }

    void CUltrasonido::startTimerLeft() {
        m_timerLeft.reset();
        m_timerLeft.start();
    }

    void CUltrasonido::stopTimerLeft() {
        m_timerLeft.stop();
        int duration = m_timerLeft.elapsed_time().count(); // Get elapsed time in microseconds
        l_distanceLeft = duration / 58.0; // Convert time to distance
    }

    void CUltrasonido::startTimerRight() {
        m_timerRight.reset();
        m_timerRight.start();
    }

    void CUltrasonido::stopTimerRight() {
        m_timerRight.stop();
        int duration = m_timerRight.elapsed_time().count(); // Get elapsed time in microseconds
        l_distanceRight = duration / 58.0; // Convert time to distance
    }

}; // namespace periodics