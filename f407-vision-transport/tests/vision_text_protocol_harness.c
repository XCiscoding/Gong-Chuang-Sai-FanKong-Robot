#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "vision.h"

static uint32_t fake_tick;
static uint32_t fake_primask;
static uint8_t rx_queue[1024];
static size_t rx_head;
static size_t rx_tail;
static char last_tx[64];
static unsigned tx_count;

uint32_t HAL_GetTick(void) { return fake_tick; }
uint32_t test_get_primask(void) { return fake_primask; }
void test_disable_irq(void) { fake_primask = 1U; }
void test_enable_irq(void) { fake_primask = 0U; }

void VisionUart_Init(void)
{
    rx_head = rx_tail = 0U;
    last_tx[0] = '\0';
    tx_count = 0U;
}

void VisionUart_IRQHandler(void) {}
void VisionUart_Service(void) {}

uint8_t VisionUart_ReadByte(uint8_t *byte)
{
    if (rx_tail == rx_head) return 0U;
    *byte = rx_queue[rx_tail++];
    return 1U;
}

uint8_t VisionUart_Send(const char *text)
{
    size_t n = strlen(text);
    assert(n < sizeof(last_tx));
    memcpy(last_tx, text, n + 1U);
    tx_count++;
    return 1U;
}

void VisionUart_GetDiagnostics(VisionUartDiagnostics *diagnostics)
{
    memset(diagnostics, 0, sizeof(*diagnostics));
}

static void feed(const char *text)
{
    size_t n = strlen(text);
    assert(rx_head + n < sizeof(rx_queue));
    memcpy(&rx_queue[rx_head], text, n);
    rx_head += n;
}

static void service_at(uint32_t tick)
{
    fake_tick = tick;
    Vision_Service();
}

int main(void)
{
    VisionObservation observation;

    fake_tick = 0U;
    Vision_Init();
    Vision_Service();
    assert(tx_count == 1U);
    assert(strcmp(last_tx, "MODE,TRACK\r\n") == 0);

    feed("ACK,TRACK\r\n");
    service_at(10U);
    assert(vision_diag.acknowledgements == 1U);
    assert(vision_diag.link_active == 1U);

    feed("TRACK,1,-120,0,50\r\n");
    service_at(20U);
    assert(Vision_GetLatest(&observation));
    assert(observation.sequence == 1U && observation.dx == -120);
    assert(!Vision_IsFresh(20U));
    assert(vision_diag.unstable_frames == 1U);

    feed("TRACK,2,40,0,100\r\n");
    service_at(30U);
    assert(Vision_IsFresh(30U));
    assert(vision_latest.sequence == 2U && vision_latest.dx == 40);

    feed("TRACK,2,999,0,100\r\nTRACK,1,888,0,100\r\n");
    service_at(40U);
    assert(vision_diag.stale_sequences == 2U);
    assert(vision_latest.sequence == 2U && vision_latest.dx == 40);

    feed("ACK,TRACK\r\nTRACK,65535,-4,0,100\r\n"
         "TRACK,0,4,0,100\r\n");
    service_at(50U);
    assert(vision_latest.sequence == 0U && vision_latest.dx == 4);
    assert(vision_diag.session_resets == 2U);

    feed("TRACK,1,LOST\r\n");
    service_at(60U);
    assert(!Vision_IsFresh(60U));
    assert(vision_diag.lost_frames == 1U);

    feed("TRACK,2,0,0,101\r\n");
    service_at(70U);
    assert(vision_diag.malformed_lines == 1U);

    service_at(261U);
    assert(vision_diag.link_timeouts == 1U);
    assert(vision_diag.link_active == 0U);
    assert(strcmp(last_tx, "MODE,TRACK\r\n") == 0);
    assert(tx_count == 2U);

    puts("vision text protocol harness: PASS");
    return 0;
}
