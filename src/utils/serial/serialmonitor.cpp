/**
 * Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*/


#include <utils/serial/serialmonitor.hpp>


namespace utils::serial{

    /** @brief  CSerialMonitor class constructor
     *
     *
     *  @param f_serialPort               reference to serial object
     *  @param f_serialSubscriberMap      map with the key and the callback functions
     */
    CSerialMonitor::CSerialMonitor(Serial& f_serialPort
                    ,CSerialSubscriberMap f_serialSubscriberMap)
            :utils::task::CTask(0)
            , m_serialPort(f_serialPort)
            , m_RxBuffer()
            , m_TxBuffer()
            , m_parseBuffer()
            , m_parseIt(m_parseBuffer.begin())
            , m_serialSubscriberMap(f_serialSubscriberMap) 
            {
                m_serialPort.attach(mbed::callback(this,&CSerialMonitor::serialRxCallback), Serial::RxIrq); 
                m_serialPort.attach(mbed::callback(this,&CSerialMonitor::serialTxCallback), Serial::TxIrq); 
            }

    /** @brief  Rx callback actions
     *  
     */
    void CSerialMonitor::serialRxCallback()
    {
        __disable_irq();
        while ((m_serialPort.readable()) && (!m_RxBuffer.isFull())) {
            char l_c = m_serialPort.getc();
            m_RxBuffer.push(l_c);
        }
        __enable_irq();
        return;
    }

    /** @brief  Tx callback actions
     *  
     */
    void CSerialMonitor::serialTxCallback()
    {
        __disable_irq();
        while ((m_serialPort.writeable()) && (!m_TxBuffer.isEmpty())) {
            m_serialPort.putc(m_TxBuffer.pop());
        }
        __enable_irq();
        return;
    }

    /** @brief  Monitoring function
     * 
     * It has role to monitor the received messaged, it applies periodically the read buffer content and decodes the message if present. 
     * Each validted message is redirectionated to the callback function corresponding to the message itself. The callback function requires two input as pointers,
     *  one for message's content and one for response's content. After the appling the callback function, it will send the response to the other device.
     */
    void CSerialMonitor::_run()
    {
        if ((!m_RxBuffer.isEmpty()))
        {
            char l_c = m_RxBuffer.pop(); // Read the next character from buffer
            if ('#' == l_c) // Message starting special character
            {
                m_parseIt = m_parseBuffer.begin();
                m_parseIt[0] = l_c;
                m_parseIt++;
                return;
            }
            if (m_parseIt != m_parseBuffer.end())
            {
                if (l_c == '\n') // Message ending character
                {
                    if ((';' == m_parseIt[-3]) && (';' == m_parseIt[-2]) && ('\r' == m_parseIt[-1])) // Check the message ending
                    {
                        char l_msgID[5];
                        char l_msg[256];

                        uint32_t res = sscanf(m_parseBuffer.data(),"#%4s:%s;;",l_msgID,l_msg); //Parse the message to key and content
                        if (res == 2) // Check the parsing
                        {
                            auto l_pair = m_serialSubscriberMap.find(l_msgID); // Search the key and callback function pair
                            if (l_pair != m_serialSubscriberMap.end()) // Check the existence of key 
                            {
                                char l_resp[256] = "no response given"; // Initial response message
                                string s(l_resp);
                                l_pair->second(l_msg,l_resp); // Apply the attached function
                                m_serialPort.printf("@%s:%s\r\n",l_msgID,l_resp); // Create the response message
                            }
                        }
                        m_parseIt = m_parseBuffer.begin(); //Go to begining of parse buffer.
                    }
                }
                m_parseIt[0] = l_c;
                m_parseIt++;
                return;
            }
        }
    }

}; // namespace serial