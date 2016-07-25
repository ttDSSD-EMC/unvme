/**
 * Copyright (c) 2015-2016, Micron Technology, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *   3. Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 * @brief NVMe header file
 */

#ifndef _UNVME_NVME_H
#define _UNVME_NVME_H

#include "nvme.h"

// Export functions
nvme_device_t* nvme_create(int mapfd);
void nvme_delete(nvme_device_t* dev);

nvme_queue_t* nvme_setup_adminq(nvme_device_t* dev, int qsize,
                                void* sqbuf, u64 sqpa, void* cqbuf, u64 cqpa);

nvme_queue_t* nvme_create_ioq(nvme_device_t* dev, int id, int qsize,
                              void* sqbuf, u64 sqpa, void* cqbuf, u64 cqpa);
int nvme_delete_ioq(nvme_queue_t* ioq);

int nvme_acmd_identify(nvme_device_t* dev, int nsid, u64 prp1, u64 prp2);
int nvme_acmd_get_log_page(nvme_device_t* dev, int nsid,
                          int lid, int numd, u64 prp1, u64 prp2);
int nvme_acmd_create_cq(nvme_queue_t* ioq, u64 prp);
int nvme_acmd_create_sq(nvme_queue_t* ioq, u64 prp);
int nvme_acmd_delete_cq(nvme_queue_t* ioq);
int nvme_acmd_delete_sq(nvme_queue_t* ioq);

int nvme_cmd_rw(nvme_queue_t* ioq, int opc, u16 cid, int nsid, u64 slba, int nlb, u64 prp1, u64 prp2);
int nvme_cmd_read(nvme_queue_t* ioq, u16 cid, int nsid, u64 slba, int nlb, u64 prp1, u64 prp2);
int nvme_cmd_write(nvme_queue_t* ioq, u16 cid, int nsid, u64 slba, int nlb, u64 prp1, u64 prp2);

int nvme_check_completion(nvme_queue_t* q, int* stat);
int nvme_wait_completion(nvme_queue_t* q, int cid, int timeout);

#endif  // _UNVME_NVME_H

