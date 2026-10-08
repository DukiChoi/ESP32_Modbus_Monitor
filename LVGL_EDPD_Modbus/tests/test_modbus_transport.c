/* Exercise the real framing/receive/batch worker with a deterministic UART. */
#include <assert.h>
#include <stdio.h>
#include "../main/modbus_src/lv_modbus_connection.c"
static TickType_t tick;
static uint32_t applied_baud;
static bool fail_speed;
int uart_set_baudrate(int port, uint32_t baud) { (void)port; applied_baud = baud; return fail_speed ? -1 : 0; }
static uint8_t stream[256];
static unsigned stream_len, stream_pos, chunk_limit = 256, reads, writes, completions;
static bool automatic, fail_first;
static modbus_data_t results[12];
static modbus_batch_t queued_batch;
TickType_t xTaskGetTickCount(void) { return tick; }
void vTaskDelay(TickType_t delay) { tick += delay; }
int xQueueSend(QueueHandle_t queue, const void *data, unsigned wait)
{
    (void)wait;
    if (queue == (void *)2) queued_batch = *(const modbus_batch_t *)data;
    else { assert(queue == (void *)1); results[completions++] = *(const modbus_data_t *)data; }
    return pdTRUE;
}
static unsigned reply(const modbus_request_t *req, uint8_t *out)
{
    out[0] = req->slave_id; out[1] = req->function_code; out[2] = req->reg_count * 2;
    for (unsigned i = 0; i < req->reg_count; i++) { out[3+i*2] = 0; out[4+i*2] = 100+i; }
    unsigned len = 3 + req->reg_count * 2;
    uint16_t crc = modbus_crc16(out, len); out[len++] = crc; out[len++] = crc >> 8;
    return len;
}
int uart_read_bytes(int port, void *buf, uint32_t length, TickType_t wait)
{
    (void)port; reads++;
    assert(length > 0 && length <= 31); /* Never wait for all 256 UART bytes. */
    unsigned n = stream_len - stream_pos;
    if (n > length) n = length;
    if (n > chunk_limit) n = chunk_limit;
    if (!n) { tick += wait; return 0; }
    memcpy(buf, stream + stream_pos, n); stream_pos += n; tick++; return n;
}
int uart_flush_input(int port) { (void)port; return 0; }
int uart_wait_tx_done(int port, TickType_t wait) { (void)port; (void)wait; return 0; }
int uart_write_bytes(int port, const char *bytes, size_t length)
{
    (void)port; assert(length == 8);
    if (automatic) {
        assert(writes == completions); /* Next wire request follows previous completion. */
        const uint8_t *f = (const uint8_t *)bytes;
        modbus_request_t req = {.slave_id=f[0], .function_code=f[1],
            .reg_addr=(f[2]<<8)|f[3], .reg_count=(f[4]<<8)|f[5]};
        stream_pos = 0;
        stream_len = fail_first && writes == 0 ? 0 : reply(&req, stream);
    }
    writes++; return length;
}
static void reset_stream(void) { stream_pos=0; tick=reads=0; chunk_limit=256; }
int main(void)
{
    modbus_request_t req = {.response_queue=(void *)1,.request_id=7,.slave_id=2,
        .function_code=3,.reg_addr=11083,.reg_count=10};
    uint8_t frame[8]; build_read_request(&req, frame);
    modbus_data_t data;
    reset_stream(); stream_len = reply(&req, stream);
    assert(receive_response(&req, frame, &data));
    assert(tick < 10 && reads == 2 && data.reg_count == 10 && data.values[9] == 109);
    reset_stream(); chunk_limit=2; stream_len=reply(&req,stream);
    assert(receive_response(&req,frame,&data) && data.values[0] == 100);
    reset_stream(); memcpy(stream,frame,8); stream_len=8+reply(&req,stream+8);
    assert(receive_response(&req,frame,&data));
    req.reg_count=1; build_read_request(&req,frame);
    reset_stream(); memcpy(stream,frame,8); stream_len=8+reply(&req,stream+8); chunk_limit=1;
    assert(receive_response(&req,frame,&data));
    reset_stream(); stream_len=reply(&req,stream); stream[stream_len-1]^=1;
    assert(!receive_response(&req,frame,&data));
    reset_stream(); reply(&req,stream); stream_len=4;
    assert(!receive_response(&req,frame,&data) && tick == MODBUS_RESP_TIMEOUT);
    reset_stream(); stream[0]=2; stream[1]=0x83; stream[2]=2;
    uint16_t crc=modbus_crc16(stream,3); stream[3]=crc; stream[4]=crc>>8; stream_len=5;
    assert(!receive_response(&req,frame,&data) && tick < 10);
    req.reg_count=13; build_read_request(&req,frame);
    reset_stream(); stream_len=reply(&req,stream); stream[2]=13;
    crc=modbus_crc16(stream,stream_len-2); stream[stream_len-2]=crc; stream[stream_len-1]=crc>>8;
    assert(receive_response(&req,frame,&data));

    modbus_batch_t batch={.count=3};
    for(unsigned i=0;i<3;i++) { batch.requests[i]=req; batch.requests[i].request_id=i+1; }
    batch.requests[0].reg_addr=11003; batch.requests[0].reg_count=6;
    batch.requests[1].reg_addr=11011; batch.requests[1].reg_count=5;
    batch.requests[2].reg_addr=11321; batch.requests[2].reg_count=1;
    reset_stream(); automatic=true; writes=completions=0;
    process_batch(&batch);
    assert(writes==3 && completions==3 && tick < 100);
    assert(results[0].valid && !results[0].batch_complete && results[2].batch_complete);
    reset_stream(); writes=completions=0; fail_first=true;
    process_batch(&batch);
    assert(!results[0].valid && results[1].valid && results[2].valid && results[2].batch_complete);
    assert(results[0].request_id==1 && results[0].reg_count==6);
    reset_stream(); writes=completions=0; fail_first=false;
    batch.baud_rate=921600; process_batch(&batch);
    assert(applied_baud==921600 && writes==3 && results[2].valid);
    reset_stream(); writes=completions=0; fail_speed=true;
    process_batch(&batch);
    assert(writes==0 && completions==3 && !results[0].valid && results[2].batch_complete);
    fail_speed=false;
    s_req_queue=(void *)2;
    assert(modbus_send_batch_at_baud(batch.requests,3,115200) && queued_batch.baud_rate==115200);
    assert(!modbus_send_batch_at_baud(batch.requests,3,12345));
    assert(modbus_send_batch(batch.requests,3) && queued_batch.count==3);
    batch.requests[0].reg_addr=65535; batch.requests[0].reg_count=2;
    assert(!modbus_send_batch(batch.requests,3));
    puts("PASS: exact-length/fragmented/echo replies, CRC/exception/timeout, immediate sequential batches, completion on failure, atomic enqueue");
    return 0;
}
