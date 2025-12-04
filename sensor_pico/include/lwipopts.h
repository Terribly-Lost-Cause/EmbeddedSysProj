#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

#define NO_SYS                 1
#define LWIP_TCPIP_CORE_LOCKING 0

#define LWIP_IPV4              1
#define LWIP_ICMP              1
#define LWIP_UDP               1

// turn THIS on:
#define LWIP_TCP               1

#define LWIP_RAW               1

// since TCP is on, give it a few PCBs
#define MEMP_NUM_TCP_PCB       5
#define TCP_QUEUE_OOSEQ        0

#define LWIP_NETCONN           0
#define LWIP_SOCKET            0
#define LWIP_NETIF_API         0
#define LWIP_COMPAT_SOCKETS    0
#define MEMP_NUM_NETCONN       0

#define LWIP_NETIF_HOSTNAME    1
#define LWIP_DHCP              1

#define MEM_ALIGNMENT          4
#define MEM_SIZE               8192
#define MEMP_NUM_PBUF          16
#define PBUF_POOL_SIZE         8

#define LWIP_ETHERNET          1
#define LWIP_ARP               1

#endif /* __LWIPOPTS_H__ */