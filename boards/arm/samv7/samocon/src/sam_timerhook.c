#include <nuttx/config.h>
#include <nuttx/board.h>
#include <semaphore.h>


#ifdef CONFIG_SYSTEMTICK_HOOK

sem_t g_waitsem;

void board_timerhook(void)
{
    sem_post(&g_waitsem);
}
#endif
