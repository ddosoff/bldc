#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define REPLY_BUF_SIZE 512
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))

uint8_t buf[REPLY_BUF_SIZE];
uint8_t *head = &buf[0];
size_t contains;

inline static void reply_buf_clear(void) 
{
    head = &buf[0];
    contains = 0;
}

/*
 * Return a pointer to one-past-the-end of the ring buffer's
 * contiguous buffer. You shouldn't normally need to use this function
 * unless you're writing a new reply_buf_* function.
 */
inline static const uint8_t * reply_buf_end(void)
{
    return buf + REPLY_BUF_SIZE;
}

inline static uint8_t * tail() {
    uint8_t *temp = head - contains;
    if (temp >= &buf[0]) {
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
    if (count > contains)
        return 0;

    size_t canBeReadTillTheEnd = MIN(reply_buf_end() - tail(), count);
    memcpy(dst, tail(), canBeReadTillTheEnd);

    size_t leftToRead = count - canBeReadTillTheEnd;
    if (leftToRead > 0) {
        memcpy(dst + canBeReadTillTheEnd, buf, leftToRead);
    }
    contains -= count;
    return 1;
}

/**
 * @brief Add [count] symbols of data to reply_buf from [src]
 * 
 * @return int 0 if cannot add data. 1 if data was added
 */
int reply_buf_append_from(const uint8_t src[], size_t count)
{
    if (count > REPLY_BUF_SIZE - contains)
        return 0;   

    const uint8_t *bufend = reply_buf_end();
    size_t appended = 0;
    size_t canBeAppendedTillTheEnd = MIN(bufend - head, count - appended);
    memcpy(head, src, canBeAppendedTillTheEnd);
    head += canBeAppendedTillTheEnd;

    if (head == bufend) {
        head = &buf[0];
    }

    size_t leftToAppend = count - canBeAppendedTillTheEnd;
    if (leftToAppend > 0) {
        memcpy(head, src + appended, leftToAppend); 
        head += leftToAppend;
    }
    contains += count;
    return 1;
}