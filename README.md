#  README

Accompanying repository to
> Tobias Hammer and Berthold Bäuml, "The Communication Layer of the aRDx Software Framework: Highly Performant and Realtime Deterministic", In Journal of Intelligent & Robotic Systems, 2014.


### Test programs

In corresponding directories

* ard/
* ardx/
* orocos/
* ros/
* yarp/

Only raw programs included, without frameworks and utility libraries.


### Test results

raw_data.dat: Test results in Mathematica-compatible format

```
{{domain, framework, clients, packet size, distributed targets, used cores,
   {avg rtt, min rtt, max rtt},
   {load core 1 .. 4},
 ...}
```

* domain
  * local: process
  * hostlocal: host
  * global: distributed
* framework
  * nomenclature
    * -bound: process bound to a single CPU core (process domain only)
    * -qnx: test run on QNX
  * frameworks used in journal article
    * ardx-small: aRDx in process domain
    * ardx-big: aRDx in host and distributed domain
    * ard: aRD
    * orocos: Orocos
    * ros-init: ROS with data initialization
    * ros: ROS (fixed) without data initialization
    * yarp: YARP
    * ardx-many-ch: aRDx (N channel)
    * sync-single-sg: aRDx (1 sync)
    * sync-multi-sg: aRDx (N sync)
    * sync-many-ch: aRDx (sync on N)
    * sync-many-ch-bound: aRDx sync on N (Linux, bound)
    * sync-many-ch-qnx: aRDx sync on N (QNX)
    * ardx-big-qnx: aRDx (QNX)
  * framework names not used in article
    * ard: ard-bound, ard-bound-qnx, ard-qnx
    * ardx with daemon: ardx-big, ardx-big-qnx, ardx-many-ch-qnx
    * ardx without daemon: ardx-small, ardx-small-bound, ardx-small-bound-qnx, ardx-small-qnx
    * ardx with sync groups: sync-many-ch-bound-qnx, sync-many-ch-qnx, sync-multi-sg-bound-qnx, sync-multi-sg-qnx, sync-single-sg-bound-qnx, sync-single-sg-qnx
* clients: number of communicating clients (C)
* packet size: in bytes
* distributed targets: number of computers running clients in distributed domain
* used cores: number of used CPU cores
* avg/min/max rtt: round-trip time in microseconds
* load: relative load on CPU core (1: idle, 0: fully used)

