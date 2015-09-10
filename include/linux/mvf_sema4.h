#ifndef __MVF_SEMA4__
#define __MVF_SEMA4__

#include <linux/sched.h>

#define MVF_SHMEM_SEMAPHORE_NUMBER      (1)
#define MVF_PRINTF_SEMAPHORE_NUMBER     (2)
#define MVF_I2C_SEMAPHORE_NUMBER        (3)
#define MVF_RESERVED1_SEMAPHORE_NUMBER  (4)
#define MVF_RESERVED2_SEMAPHORE_NUMBER  (5)

#ifdef __KERNEL__

typedef struct mvf_sema4_handle_struct {
	int gate_num;
	wait_queue_head_t wait_queue;
	// stats
	u32 attempts;
	u32 interrupts;
	u32 failures;
	struct timeval request_time;
	u64 total_latency_us;
	u32 worst_latency_us;
} MVF_SEMA4;

int mvf_sema4_assign(int gate_num, MVF_SEMA4** sema4_p);
int mvf_sema4_deassign(MVF_SEMA4 *sema4);
int mvf_sema4_lock(MVF_SEMA4 *sema4, unsigned int timeout_us, bool use_interrupts);
int mvf_sema4_unlock(MVF_SEMA4 *sema4);
int mvf_sema4_test(MVF_SEMA4 *sema4);

#endif
#endif
