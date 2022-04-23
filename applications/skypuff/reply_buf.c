#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define REPLY_BUF_SIZE 512
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))

uint8_t reply_buf[REPLY_BUF_SIZE];
uint8_t *reply_buf_head = reply_buf;
size_t reply_buf_contains;

inline static void reply_buf_clear(void)
{
    reply_buf_head = reply_buf;
    reply_buf_contains = 0;
}

/*
 * Return a pointer to one-past-the-end of the ring buffer's
 * contiguous buffer. You shouldn't normally need to use this function
 * unless you're writing a new reply_buf_* function.
 */
inline static const uint8_t * reply_buf_end(void)
{
    return reply_buf + REPLY_BUF_SIZE;
}

inline static uint8_t * reply_buf_tail(void) {
    uint8_t *temp = reply_buf_head - reply_buf_contains;
    if (temp >= reply_buf) {
        return temp;
    }
    return temp + REPLY_BUF_SIZE;
}

/**
 * @brief Read [count] symbols of data to [dst] from reply_buf
 *
 * @return int 0 if cannot read data. 1 if data was read
 */
int reply_buf_read_to(uint8_t dst[], size_t count)
{
    if (count > reply_buf_contains)
        return 0;

    size_t canBeReadTillTheEnd = MIN(reply_buf_end() - reply_buf_tail(), count);
    memcpy(dst, reply_buf_tail(), canBeReadTillTheEnd);

    size_t leftToRead = count - canBeReadTillTheEnd;
    if (leftToRead > 0) {
        memcpy(dst + canBeReadTillTheEnd, reply_buf, leftToRead);
    }
    reply_buf_contains -= count;
    return 1;
}

/**
 * @brief Add [count] symbols of data to reply_buf from [src]
 *
 * @return int 0 if cannot add data. 1 if data was added
 */
int reply_buf_append_from(const uint8_t src[], size_t count)
{
    if (count > REPLY_BUF_SIZE - reply_buf_contains)
        return 0;

    const uint8_t *bufend = reply_buf_end();
    size_t canBeAppendedTillTheEnd = MIN(bufend - reply_buf_head, count);
    memcpy(reply_buf_head, src, canBeAppendedTillTheEnd);
    reply_buf_head += canBeAppendedTillTheEnd;

    if (reply_buf_head == bufend) {
        reply_buf_head = reply_buf;
    }

    size_t leftToAppend = count - canBeAppendedTillTheEnd;
    if (leftToAppend > 0) {
        memcpy(reply_buf_head, src + canBeAppendedTillTheEnd, leftToAppend);
        reply_buf_head += leftToAppend;
    }
    reply_buf_contains += count;
    return 1;
}