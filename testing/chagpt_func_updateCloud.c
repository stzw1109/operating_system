void func_updateCloud(void *argument)
{
  /* USER CODE BEGIN func_updateCloud */
  /* Infinite loop */
  for(;;)
  {
	  //CHATGPT func update cloud
	  mqttnetwork_connect(&network, "broker.hivemq.com", 1883); 

	   MQTTClientInit(&client, &network, 5000, sendbuf, sizeof(sendbuf), readbuf, sizeof(readbuf));

	   connectData.MQTTVersion = 3;
	   connectData.clientID.cstring = "STM32Client";
	   connectData.username.cstring = NULL;
	   connectData.password.cstring = NULL;
	   
	   if (MQTTConnect(&client, &connectData) != SUCCESS)
	   {
	     ITM_SendChar('F');  // connection failed
	     vTaskDelete(NULL);
	   }

	   MQTTMessage message;
	   message.qos = QOS0;
	   message.retained = 0;
	   message.payload = sendbuf;

	   while (1)
	   {
	     snprintf((char *)sendbuf, sizeof(sendbuf), "Petrol Volume: %lu", petrol_tank_volume);
	     message.payloadlen = strlen((char *)sendbuf);

	     MQTTPublish(&client, "stm32/petrol", &message);

	     osDelay(5000); // Wait 5 seconds
	   }
  }
  /* USER CODE END func_updateCloud */
}
