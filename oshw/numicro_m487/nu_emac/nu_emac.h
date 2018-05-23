/*
 * author: Wayne Lin
 */

#ifndef NU_EMAC_H
#define NU_EMAC_H

int NU_EMAC_init(uint8_t *enetaddr);
int NU_EMAC_send(void *packet, int length);
int NU_EMAC_recv(uint8_t * packet, size_t size);

#endif /* NU_EMAC_H */
