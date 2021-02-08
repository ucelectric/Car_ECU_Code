import can

#create a bus instance
bus = can.Bus(interface = 'socketcan',
              channel='wlan0',
              receive_own_message=False)

#send message
#message = can.Message(arbitration_id=123, is_extended_id=True,
                      #data=[0x11, 0x22, 0x33])
#bus.send(message, timeout=0.2)
                      
#iterate over received messages
for msg in bus:
    print("{X}: {}".format(msg.arbitration_id, msg.data))
    
#or use an asynchronous notifier
#notifier = can.Notifier(bus, [can.Logger("recorded.log"), can.Printer()])