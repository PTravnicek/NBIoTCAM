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


## Completed
- [x] Speed up the connection process and photo sending by checking the status and proceeding the AT commands ASAP
- [x] Fix TCP socket creation errors
- [x] Some problems with UDP message when TCP is not sent. (module was not initialized witout the photo)
- [x] Check the propper connection and registration to the network before trying to send anything
- [x] Solve REBOOT_CAUSE_SECURITY_PMU_POWER_ON_RESET, problably power supply problem. / no issue when power from bench/ issue not present without direct solution (??)
- [x] Timeout for the photo transfer on node-red.
- [x] Accept only traffic from the actual photo tcp transfer (opening and closing bytes). 
- [x] Sending the photo over TCP reliably- checking for errors ATS+NOCL - IF error, retry sending command again.
- [x] In node-red, send the UDP message to the RM server.

## Notes
- HW modding the BC95 module and power it from the 3.3V pin instead of the LDO. - doesn't work, getting no connection to the server, but the same issue has been present after "un-modding" the module. Resolved on its own after a while.

