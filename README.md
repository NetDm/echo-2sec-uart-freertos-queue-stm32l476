# echo-2sec-uart-freertos-queue-stm32l476

Проект stm32cube freeRTOS Queue 


Эхо UART с задержкой 2сек

В прерывании пишет в очеред freeRTOS пишет структуру, состоящую из принятых данных UART (1 байта) и системного времени (в милисекундах) его прихода

В основном таске - забирает данные с времеными штамами и помещает в большой кольцевой буфер,
далее данные из большого буфера сверяются с текущим системным временем и по истечении 2 сек отдаются обратно в UART

атомарность временных интервалов и порядка данных соблюдается в пределах 1 мс на 2 сек шкале
