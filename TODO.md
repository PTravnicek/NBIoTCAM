# TODO List

## In Progress
- [ ] Downlink messages/ receive command.
        4.8. AT+NSORF Receive Command
        When data arrives, a “+NSONMI” response will be generated to indicate the socket the message was received on and also the amount of data
        UDP Message: You can create a socket, send, receive, and read UDP messages. After the socket is closed, no response is received.
            ◦ Disable registration for Huawei’s IoT platform: AT+QREGSWT=2
            ◦ Query the IP address of the module: AT+CGPADDR
            ◦ Create a socket: AT+NSOCR=DGRAM,17,0,1
            ◦ Send a message: AT+NSOST=1,220.180.239.212,8012,5,1245783132,100
            ◦ Datagram is sent over RF: +NSOSTR:1,100,1
            ◦ Received the message: +NSONMI:1,5
            ◦ Read the message: AT+NSORF=1,5
            ◦ Close the socket: AT+NSOCL=1
        Receive the IP address of the module on the Node-red platform and use it for the UDP downlink message


## Upcoming
- [ ] Some problems with UDP message when TCP is not sent. 
        Problem: NO RESPONSE FROM THE MODULE
            [BC95 SEND] AT+NRB
            [BC95 SEND] AT+CFUN=1
            [BC95 SEND] AT+CGATT=1
            [BC95 SEND] AT+QREGSWT=2
            Line Protocol as ASCII:
            gps_data,device=esp32 latitude=50.123400,longitude=14.123400,altitude=123.4,speed=0.00,course=0.00,satellites=0,battery=42

            Line Protocol as HEX:
            6770735F646174612C6465766963653D6573703332206C617469747564653D35302E3132333430302C6C6F6E6769747564653D31342E3132333430302C616C7469747564653D3132332E342C73706565643D302E30302C636F757273653D302E30302C736174656C6C697465733D302C626174746572793D3432
            [BC95 SEND] AT+NSOCR=DGRAM,17,0,1


            Parsed UDP socket ID: -1
            Failed to parse UDP socket ID!
        HOW it should look like:
            [BC95 SEND] AT+NRB
            [BC95 RSP] REBOOTING

            REBOOT_CAUSE_APPLICATION_AT
            Neul 
            OK
            [BC95 SEND] AT+CFUN=1
            [BC95 RSP] OK
            [BC95 SEND] AT+CGATT=1
            [BC95 RSP] OK
            [BC95 SEND] AT+QREGSWT=2
            [BC95 RSP] OK
            Line Protocol as ASCII:
            gps_data,device=esp32 latitude=50.123400,longitude=14.123400,altitude=123.4,speed=0.00,course=0.00,satellites=0,battery=42

            Line Protocol as HEX:
            6770735F646174612C6465766963653D6573703332206C617469747564653D35302E3132333430302C6C6F6E6769747564653D31342E3132333430302C616C7469747564653D3132332E342C73706565643D302E30302C636F757273653D302E30302C736174656C6C697465733D302C626174746572793D3432
            [BC95 SEND] AT+NSOCR=DGRAM,17,0,1
            [BC95 RSP] 0
- [ ] Fix TCP socket creation errors:
        REBOOT_CAUSE_APPLICATION_AT
        Neul 
        OK
        [BC95 SEND] AT+CFUN=1
        [BC95 RSP] OK
        [BC95 SEND] AT+CGATT=1
        [BC95 RSP] OK
        [BC95 SEND] AT+QREGSWT=2
        [BC95 RSP] OK
        [BC95 SEND] AT+NSOCR=STREAM,6,0,1
        Failed to create TCP socket!


## Completed
- [x] Solve REBOOT_CAUSE_SECURITY_PMU_POWER_ON_RESET, problably power supply problem. / no issue when power from bench/ issue not present without direct solution (??)
- [x] Timeout for the photo transfer on node-red.
- [x] Accept only traffic from the actual photo tcp transfer (opening and closing bytes). 
- [x] Sending the photo over TCP reliably- checking for errors ATS+NOCL - IF error, retry sending command again.
- [x] In node-red, send the UDP message to the RM server.

## Notes
- HW modding the BC95 module and power it from the 3.3V pin instead of the LDO. - doesn't work, getting no connection to the server, but the same issue has been present after "un-modding" the module. Resolved on its own after a while.

