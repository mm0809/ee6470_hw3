# E6470 HW2

## Gaussian blur with TLM

### How to run

```bash
$ mkdir build && cd build
$ cmake .. && make run
```

### Different with HW1
In HW1 we declare FIFO in `main.cpp` and connect
'sc_fifo_in/out' in Testbench and Sobelfilter 
in pair.

In HW2 with TLM, `Initiator::read\write_from_socket`
trigger `Sobelfilter::blocking_transport` and pass
the data back by pointer. We don't need FIFO the 
transmit data but store data in order, so we only
need to declare FIFO in Sobelfilter.
