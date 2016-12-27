
void receive(const MyMessage &message) {
  // We only expect one type of message from controller. But we better check anyway.
  if (message.isAck()) {
     Serial.println("This is an ack from gateway");
  }
  state = message.getBool();      // Change relay state
  if (message.type == V_LIGHT && message.sensor == CHILD_ID_Relay_1) {
       Serial.println("Drive Relay 1");
       digitalWrite(Relay_1, state);

       // Write some debug info
       Serial.print("Incoming change for sensor:");
       Serial.print(message.sensor);
       Serial.print(", New status: ");
       Serial.println(state);
   } 
   else if (message.type == V_LIGHT && message.sensor == CHILD_ID_Relay_2) {
       Serial.println("Drive Relay 2");
       digitalWrite(Relay_2, state);
       // Write some debug info
       Serial.print("Incoming change for sensor:");
       Serial.print(message.sensor);
       Serial.print(", New status: ");
       Serial.println(state);
        }
     else  {};

}
