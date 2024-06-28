#include "RTProtocol.h"
#include "RTPacket.h"
#include <iostream>
#include <chrono>

using namespace std::chrono;

#ifdef _WIN32
#define sleep Sleep
#else
#include <unistd.h>
#endif

int main(int argc, char **argv)
{
    try
    {
        CRTProtocol rtProtocol;

        //Example code for how to use discovery calls.
        //if (rtProtocol.DiscoverRTServer(4534, false))
        //{
        //    sleep(1);
        //    const auto numberOfResponses = rtProtocol.GetNumberOfDiscoverResponses();
        //    for (auto index = 0; index < numberOfResponses; index++)
        //    {
        //        unsigned int addr;
        //        unsigned short basePort;
        //        std::string message;
        //        if (rtProtocol.GetDiscoverResponse(index, addr, basePort, message))
        //        {
        //            printf("%2d - %d.%d.%d.%d:%d\t- %s\n", index, 0xff & addr, 0xff & (addr >> 8), 0xff & (addr >> 16), 0xff & (addr >> 24), basePort, message.c_str());
        //        }
        //    }
        //}
        //else
        //{
        //    printf("%s", rtProtocol.GetErrorString());
        //}

        const char           serverAddr[] = "192.168.1.216";
        const unsigned short basePort = 22222;
        const int            majorVersion = 1;
        const int            minorVersion = 19;
        const bool           bigEndian = false;

        bool dataAvailable = false;
        bool streamFrames = false;
        unsigned short udpPort = 6734;

        auto start = high_resolution_clock::now();
        double freq = 0.0;
        int j;
        while (true)
        {
            j++;
            if (!rtProtocol.Connected())
            {
                if (!rtProtocol.Connect(serverAddr, basePort, &udpPort, majorVersion, minorVersion, bigEndian))
                {
                    printf("rtProtocol.Connect: %s\n\n", rtProtocol.GetErrorString());
                    sleep(1);
                    continue;
                }
            }

            if (!dataAvailable)
            {
                if (!rtProtocol.Read6DOFSettings(dataAvailable))
                {
                    printf("rtProtocol.Read6DOFSettings: %s\n\n", rtProtocol.GetErrorString());
                    sleep(1);
                    continue;
                }
            }

            if (!streamFrames)
            {
                if (!rtProtocol.StreamFrames(CRTProtocol::RateAllFrames, 0, udpPort, NULL, CRTProtocol::cComponent6d))
                {
                    printf("rtProtocol.StreamFrames: %s\n\n", rtProtocol.GetErrorString());
                    sleep(1);
                    continue;
                }
                streamFrames = true;

                printf("Starting to streaming 6DOF data\n\n");
            }

            CRTPacket::EPacketType packetType;

            if (rtProtocol.ReceiveRTPacket(packetType, true) > 0)
            {
                if (packetType == CRTPacket::PacketData)
                {
                    float fX, fY, fZ;
                    float rotationMatrix[9];

                    CRTPacket* rtPacket = rtProtocol.GetRTPacket();

                    printf("Frame %d\n", rtPacket->GetFrameNumber());
                    printf("======================================================================================================================\n");

                    for (unsigned int i = 0; i < rtPacket->Get6DOFBodyCount(); i++)
                    {
                        if (rtPacket->Get6DOFBody(i, fX, fY, fZ, rotationMatrix))
                        {
                            const char* pTmpStr = rtProtocol.Get6DOFBodyName(i);
                            if (pTmpStr)
                            {
                                printf("%-12s ", pTmpStr);
                            }
                            else
                            {
                                printf("Unknown     ");
                            }
                            if(freq != 0.0){
                                std::cout << "Average Frequency: " << freq << " Hz" << std::endl;
                            }
                            printf("Pos: %9.3f %9.3f %9.3f    Rot: %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f\n",
                                fX, fY, fZ, rotationMatrix[0], rotationMatrix[1], rotationMatrix[2],
                                rotationMatrix[3], rotationMatrix[4], rotationMatrix[5], rotationMatrix[6], rotationMatrix[7], rotationMatrix[8]);
                        }
                    }
                    printf("\n");
                    auto current = high_resolution_clock::now();
                    duration<double> elapsed = current - start;
                    double averageFrequency = j / elapsed.count();

                    freq = averageFrequency;
                }
            }
        }
        rtProtocol.StopCapture();
        rtProtocol.Disconnect();
    }
    catch (std::exception& e)
    {
        printf("%s\n", e.what());
    }
    return 1;
}