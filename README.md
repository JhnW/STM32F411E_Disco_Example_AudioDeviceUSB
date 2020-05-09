Core information

Very basic, example project for STM32F411E-DISCO. It show how to use build-in on board audio I/O and connect they signal source to USB I/O as USB Audio Class 2.0 device.
This project its intentionally simple  to let you read modify and read code very fast. It is strongly compatibility with STM32CubeMX code generator (unfortunately, full support isn't possible) to let you regenerate wire aspect of project with only minor changes. This also used BSP library as standard solution of hardware access.

------------

How its works
- Just flash your board with this program.
- Connect ST-Link USB to power on bard (or use different supply option) and plug USB OTG connector.
- Plug headphones to board.  Please, pay attention about headphones hardware requirements. You can find it in your board documentation.
- At the start board will play sound to you headphones as external usb audio card (48kHz).
- Press user button to mute audio output and start recording audio signal (16 kHz) as usb microphone.
- Press user button again to back audio play.
- Unused audio mode will play/record zero signal - silence.

I tested this solution only on Linux system (Ubuntu 18.04). Microsoft system don't like USB Audio Class so it couldn't work out-of box.

------------

Features
- USB descriptor for two audio terminals, one for input and one for output. I hope it will be valuable example.
- BSP and ST USB library example how to play, record, send and get data.
- STM32CubeMX compatibility.
- Just can use it as external audio output card or USB microphone like last resort ;)

------------

Know issues
- Sometimes switch mode will not activate audio output.
- Windows 10 not tested.
- Audio quality may be bad. Good synchronization of device and usb clock domain is complex task who will obscure code.
- USB data transfer don't use DMA feature so it need to be stopped (USb speaker handle) during we are in microphone mode.
- Only simple loop-count key debouncing.

------------

FAQ
- Why do you make this example?
Because I will learn somethings about USB and audio handling in MCU (second much).         Unfortunately I don't fight many examples, explanations or tutorials. Many sources was only datasheets, examples for others platform or old project, hard to implemented with new HAL libraries (for example Russian USB microphone project for STM32 in the abyss of the network). So now you are able to save your time, at least until this project becomes outdated.
- Why we can't play and record in the same time?
Please check BSP init clock configuration for audio input and output. Event with the same frequency (for example, 16 kHz) I2S clock need different configuration.
Additional, without DMA in USB data transfer, handle all cases will impact on audio delay and make additional noises. Maybe RTOS will solve the second problem.
- So why do you not use DMA in USB driver? Or other improvements?
Because I'm lazy. It is very simple example, just be happy that you can read code extreme fast if you are familiar with standard STM libraries.


------------

Last worlds

Don't be heisted to pull request. Just please try don't break code-generation compatibility when it is not necessary. Improvement should be simple, to do not expanded this project much.
If you have any question, just write to me directly.
Have fun.
