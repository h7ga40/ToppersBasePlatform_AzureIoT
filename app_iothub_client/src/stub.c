#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/time.h>
#include <kernel.h>
#include <t_syslog.h>
#include "main.h"
#include "kernel_cfg.h"

void __wrap_exit(int status)
{
	ext_ker();
}

int _kill(int pid, int sig)
{
	return 0;
}

int _getpid(int n)
{
	return 1;
}

int custom_rand_generate_seed(uint8_t *output, uint32_t sz)
{
	SYSTIM now;
	int32_t i;

	get_tim(&now);
	srand(now);

	for (i = 0; i < sz; i++)
		output[i] = rand();

	return 0;
}

extern uint32_t heap_area[];

void *_sbrk(int incr)
{
    static unsigned char *heap = (unsigned char *)heap_area;
    unsigned char        *prev_heap = heap;
    unsigned char        *new_heap = heap + incr;

    if (new_heap >= (unsigned char *)&heap_area[(int)heap_param[1] / sizeof(uint32_t)]) {
        errno = ENOMEM;
        return (void *) -1;
    }

    heap = new_heap;
    return (void *) prev_heap;
}

int malloc_lock_sem_count[TNUM_TSKID];

__attribute__((weak))
void __malloc_lock(struct _reent *_r)
{
	ER ercd;
	ID tskid = 0;

	ercd = get_tid(&tskid);
	if (ercd != E_OK) {
		goto error;
	}
	tskid--;
	if ((tskid < 0) || (tskid >= TNUM_TSKID)){
		goto error;
	}

	if (malloc_lock_sem_count[tskid] == 0) {
		ercd = wai_sem(TLSF_SEM);
		if (ercd != E_OK) {
			goto error;
		}
	}

	malloc_lock_sem_count[tskid]++;
	return;
error:
	syslog(LOG_ERROR, "%s (%d) __malloc_lock error.",
		itron_strerror(ercd), SERCD(ercd));
	asm("bkpt #0");
	while(0);
}

__attribute__((weak))
void __malloc_unlock(struct _reent *_r)
{
	ER ercd;
	ID tskid = 0;
	int count;

	ercd = get_tid(&tskid);
	if (ercd != E_OK) {
		goto error;
	}
	tskid--;
	if ((tskid < 0) || (tskid >= TNUM_TSKID)){
		goto error;
	}

	malloc_lock_sem_count[tskid]--;
	if (malloc_lock_sem_count[tskid] > 0)
		return;

	if (malloc_lock_sem_count[tskid] < 0) {
		goto error;
	}

	ercd = sig_sem(TLSF_SEM);
	if (ercd != E_OK) {
		goto error;
	}
	return;
error:
	syslog(LOG_ERROR, "%s (%d) __malloc_unlock error.",
		itron_strerror(ercd), SERCD(ercd));
	asm("bkpt #0");
	while(0);
}
