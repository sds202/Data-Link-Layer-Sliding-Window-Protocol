#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <windows.h>

#include "protocol.h"
#include "datalink.h"

#define DATA_TIMER 2000 // Data帧超时
#define ACK_TIMER 270   // Ack帧超时
#define MAX_SEQ 31
#define NR_BUFS ((MAX_SEQ + 1) / 2) // 窗口大小 (必须是偶数)
#define MAX_WINDOW_SIZE NR_BUFS

typedef unsigned char seq_nr;

/* FRAME kind */
#define FRAME_DATA 0
#define FRAME_ACK 1
#define FRAME_NAK 2

#define inc(k)       \
    if (k < MAX_SEQ) \
        k++;         \
    else             \
        k = 0

struct FRAME
{
    unsigned char kind;
    seq_nr ack;
    seq_nr seq;
    unsigned char data[PKT_LEN];
    unsigned int padding;
};

static bool between(seq_nr a, seq_nr b, seq_nr c) // 判断序号是否在窗口内
{
    return ((a <= b && b < c) || (c < a && a <= b) || (b < c && c < a));
}

// --- 全局变量 ---
static seq_nr next_frame_to_send = 0; // Send方下一个要Send的帧序号
static seq_nr ack_expected = 0;       // Send方下一个要确认的帧序号
static seq_nr frame_expected = 0;     // 接收方下一个要接收的帧序号
static seq_nr too_far;                // 接收方下一个要接收的帧序号 (窗口上界)

static unsigned char out_buf[NR_BUFS][PKT_LEN]; // Send方缓冲区
static unsigned char in_buf[NR_BUFS][PKT_LEN];  // 接收方缓冲区
static bool arrived[NR_BUFS];                   // 接收方缓冲区位图 (标记哪些槽已填充)

static int nbuffered = 0;  // Send方缓冲区中已存放的帧数
static int phl_ready = 1;  // 物理层是否准备好接收数据
static bool no_nak = true; // 是否禁止连续Send NAK

static void put_frame(unsigned char *frame, int len) // Send帧到物理层
{
    *(unsigned int *)(frame + len) = crc32(frame, len);
    send_frame(frame, len + 4);
    phl_ready = 0;
}

/* Send数据帧 */
static void send_data_frame(seq_nr frame_nr)
{
    struct FRAME s;
    s.kind = FRAME_DATA;
    s.seq = frame_nr;
    s.ack = (frame_expected + MAX_SEQ) % (MAX_SEQ + 1);   // Send方下一个要确认的帧序号
    memcpy(s.data, out_buf[frame_nr % NR_BUFS], PKT_LEN); // 将数据拷贝到帧中

    dbg_frame("Send DATA %d %d, ID %d\n", s.seq, s.ack, *(short *)s.data);
    put_frame((unsigned char *)&s, 3 + PKT_LEN);
    start_timer(frame_nr % NR_BUFS, DATA_TIMER); // 启动数据帧计时器
    stop_ack_timer();
}

/* Send ACK 帧 */
// 与 send_data_frame 类似，但Send ACK 帧时不需要携带数据，且序号字段未使用
static void send_ack_frame(void)
{
    struct FRAME s;
    s.kind = FRAME_ACK;
    s.ack = (frame_expected + MAX_SEQ) % (MAX_SEQ + 1);
    s.seq = next_frame_to_send;

    dbg_frame("Send ACK %d\n", s.ack);
    put_frame((unsigned char *)&s, 2);
    stop_ack_timer();
}

/* Send NAK 帧 */
// NAK与ACK类似，但ACK帧的ack字段是下一个要确认的帧序号，而NAK帧的ack字段是下一个要接收的帧序号
static void send_nak_frame(void)
{
    struct FRAME s;
    s.kind = FRAME_NAK;

    s.ack = (frame_expected + MAX_SEQ) % (MAX_SEQ + 1);
    s.seq = 0; // seq 字段未使用

    no_nak = false; // 抑制连续 NAK

    dbg_frame("Send NAK (ack=%d)\n", s.ack);
    put_frame((unsigned char *)&s, 2); // NAK 帧长度为 2 (kind + ack)
    stop_ack_timer();
}

int main(int argc, char **argv)
{
    SetConsoleOutputCP(65001);
    int event;
    seq_nr arg; // 接收帧
    struct FRAME f;
    int len = 0;

    protocol_init(argc, argv);
    lprintf("SR-3, 构建时间: " __DATE__ "  " __TIME__ "\n");

    // 初始化
    too_far = NR_BUFS;
    nbuffered = 0;
    phl_ready = 0;
    no_nak = true;
    memset(arrived, 0, sizeof(arrived));

    disable_network_layer();

    for (;;)
    {
        // --- 修改: 将 wait_for_event 的参数解释为 seq_nr ---
        event = wait_for_event((int *)&arg); // 强制类型转换，假设返回的是序号

        switch (event)
        {
        case NETWORK_LAYER_READY:
            if (nbuffered < NR_BUFS)
            {
                // 正常接收数据并存入Send缓冲区
                get_packet(out_buf[next_frame_to_send % NR_BUFS]);
                nbuffered++;
                send_data_frame(next_frame_to_send);
                inc(next_frame_to_send);
            }
            break;

        case PHYSICAL_LAYER_READY:
            phl_ready = 1;
            break;

        case FRAME_RECEIVED:
            len = recv_frame((unsigned char *)&f, sizeof f);

            if (len < 5 || crc32((unsigned char *)&f, len) != 0)
            {
                dbg_event("**** CRC错误\n");
                if (no_nak)
                {
                    // 校验错误且当前没有NAK时，SendNAK
                    send_nak_frame();
                }
                break;
            }

            if (f.kind == FRAME_DATA)
            {
                dbg_frame("收到 DATA %d %d, ID %d\n", f.seq, f.ack, *(short *)f.data);
                dbg_frame("frame_expected = %d, too_far = %d\n", frame_expected, too_far);
                // 非预期帧序列号时SendNAK
                if (f.seq != frame_expected && no_nak)
                {
                    send_nak_frame();
                }
                else
                {
                    // 收到预期帧时，停止ACK计时器
                    start_ack_timer(ACK_TIMER);
                }

                if (between(frame_expected, f.seq, too_far)) // 序号在窗口内
                {
                    if (!arrived[f.seq % NR_BUFS]) // 窗口未满时
                    {

                        arrived[f.seq % NR_BUFS] = true;
                        memcpy(in_buf[f.seq % NR_BUFS], f.data, PKT_LEN);

                        while (arrived[frame_expected % NR_BUFS])
                        {
                            // 按序提交网络层，因此实际上不是一并Send
                            put_packet(in_buf[frame_expected % NR_BUFS], PKT_LEN);

                            no_nak = true;
                            arrived[frame_expected % NR_BUFS] = false;
                            inc(frame_expected);
                            inc(too_far);
                            start_ack_timer(ACK_TIMER);
                        }
                    }
                    else
                    {
                        // 这里拆分了逻辑，在收到过ACK后直接重置时钟
                        dbg_frame("DATA %d 已收到过\n", f.seq);
                        start_ack_timer(ACK_TIMER);
                    }
                }
                else
                {
                    // 再一次判断
                    dbg_frame("DATA %d 在窗口外 [%d, %d)\n", f.seq, frame_expected, too_far);
                    start_ack_timer(ACK_TIMER);
                }
            }

            // --- NAK 处理
            if (f.kind == FRAME_NAK)
            {
                seq_nr missing_seq = (f.ack + 1) % (MAX_SEQ + 1); // 从 ack 推断丢失帧
                dbg_frame("收到 NAK (ack=%d), 推断丢失 %d\n", f.ack, missing_seq);

                // 这里又进行判断重传帧是否在当前窗口
                if (between(ack_expected, missing_seq, next_frame_to_send))
                {
                    dbg_frame("重传帧 %d (因 NAK)\n", missing_seq);
                    send_data_frame(missing_seq);
                }
                else
                {
                    dbg_frame("推断丢失的帧 %d 不在窗口 [%d, %d) 内, 忽略 NAK\n", missing_seq, ack_expected, next_frame_to_send);
                }
                // break; // SR-2 没有 break，允许处理捎带确认
            }

            // 添加一个ACK接收的调试警告
            if (f.kind == FRAME_ACK)
            {
                dbg_frame("收到 ACK %d\n", f.ack);
            }

            // 处理确认信息
            // 判断是否在当前窗口内，如果在窗口内，则滑动窗口
            while (between(ack_expected, f.ack, next_frame_to_send))
            {
                nbuffered--;
                stop_timer(ack_expected % NR_BUFS);
                inc(ack_expected);
            }
            break;

        // 主要修改了重传检测帧在不在当前窗口的逻辑
        case DATA_TIMEOUT:
        {
            dbg_event("---- DATA %d 超时 (来自 wait_for_event)\n", arg);
            if (between(ack_expected, arg, next_frame_to_send))
            {
                dbg_event("---- 重传超时的帧 %d\n", arg);
                send_data_frame(arg);
            }
            else
            {

                dbg_event("---- 超时帧 %d 不在当前窗口 [%d, %d) 内, 暂不重传\n", arg, ack_expected, next_frame_to_send);
                send_data_frame(arg + NR_BUFS); // 这里是为了避免死循环，直接重传窗口外的帧
            }
        }
        break;

        case ACK_TIMEOUT:
            dbg_event("---- ACK超时\n");
            send_ack_frame();
            break;
        }

        if (nbuffered < NR_BUFS && phl_ready)
            enable_network_layer();
        else
            disable_network_layer();
    }
}