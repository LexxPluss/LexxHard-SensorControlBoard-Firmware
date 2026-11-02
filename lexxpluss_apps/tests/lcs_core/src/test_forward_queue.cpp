/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <array>
#include <cstddef>
#include <cstdint>

#include <zephyr/ztest.h>

#include "lcs_can_packer.hpp"
#include "lcs_forward_queue.hpp"

namespace lcs = lexxhard::lcs;

namespace {

/* Bounded FIFO fake. The first `fail_first` try_put calls fail regardless of room. */
template <std::size_t N>
class FakeQueue {
public:
    explicit FakeQueue(std::size_t fail_first = 0) : fail_first_{fail_first} {}

    bool try_put(const lcs::msg &frame)
    {
        ++try_put_calls;
        if (try_put_calls <= fail_first_ || count_ == N)
            return false;
        frames_[count_++] = frame;
        return true;
    }

    void purge()
    {
        ++purge_calls;
        count_ = 0;
    }

    std::size_t size() const { return count_; }
    const lcs::msg &at(std::size_t index) const { return frames_[index]; }

    std::size_t try_put_calls{0};
    std::size_t purge_calls{0};

private:
    std::array<lcs::msg, N> frames_{};
    std::size_t count_{0};
    std::size_t fail_first_;
};

lcs::msg make_frame(uint8_t board, uint8_t first)
{
    lcs::msg frame{};
    frame.board = board;
    frame.data[0] = first;
    return frame;
}

}

ZTEST_SUITE(lcs_forward_queue, NULL, NULL, NULL, NULL, NULL);

ZTEST(lcs_forward_queue, test_not_full_keeps_all)
{
    FakeQueue<3> queue;
    queue.try_put(make_frame(0, 0x11));
    queue.try_put(make_frame(1, 0x22));
    queue.try_put_calls = 0;

    lcs::put_purging(queue, make_frame(0, 0x33));

    zassert_equal(queue.size(), 3);
    zassert_equal(queue.at(0).data[0], 0x11);
    zassert_equal(queue.at(1).data[0], 0x22);
    zassert_equal(queue.at(2).data[0], 0x33);
    zassert_equal(queue.purge_calls, 0);
    zassert_equal(queue.try_put_calls, 1);
}

ZTEST(lcs_forward_queue, test_full_purges_then_stores_new)
{
    FakeQueue<3> queue;
    queue.try_put(make_frame(0, 0x11));
    queue.try_put(make_frame(1, 0x22));
    queue.try_put(make_frame(0, 0x33));
    queue.try_put_calls = 0;

    lcs::put_purging(queue, make_frame(0, 0x44));

    zassert_equal(queue.size(), 1);
    zassert_equal(queue.at(0).data[0], 0x44);
    zassert_equal(queue.purge_calls, 1);
    zassert_equal(queue.try_put_calls, 2);
}

ZTEST(lcs_forward_queue, test_repeated_overflow_sequence)
{
    FakeQueue<2> queue;

    lcs::put_purging(queue, make_frame(0, 0x01));
    lcs::put_purging(queue, make_frame(0, 0x02));
    zassert_equal(queue.size(), 2);
    zassert_equal(queue.at(0).data[0], 0x01);
    zassert_equal(queue.at(1).data[0], 0x02);

    lcs::put_purging(queue, make_frame(0, 0x03));
    zassert_equal(queue.size(), 1);
    zassert_equal(queue.at(0).data[0], 0x03);

    lcs::put_purging(queue, make_frame(0, 0x04));
    zassert_equal(queue.size(), 2);
    zassert_equal(queue.at(0).data[0], 0x03);
    zassert_equal(queue.at(1).data[0], 0x04);

    lcs::put_purging(queue, make_frame(0, 0x05));
    zassert_equal(queue.size(), 1);
    zassert_equal(queue.at(0).data[0], 0x05);

    zassert_equal(queue.purge_calls, 2);
}

ZTEST(lcs_forward_queue, test_retry_until_success)
{
    {
        FakeQueue<4> queue{0};
        lcs::put_purging(queue, make_frame(0, 0x77));
        zassert_equal(queue.purge_calls, 0);
        zassert_equal(queue.try_put_calls, 1);
        zassert_equal(queue.at(queue.size() - 1).data[0], 0x77);
    }
    {
        FakeQueue<4> queue{1};
        lcs::put_purging(queue, make_frame(0, 0x77));
        zassert_equal(queue.purge_calls, 1);
        zassert_equal(queue.try_put_calls, 2);
        zassert_equal(queue.at(queue.size() - 1).data[0], 0x77);
    }
    {
        FakeQueue<4> queue{2};
        lcs::put_purging(queue, make_frame(0, 0x77));
        zassert_equal(queue.purge_calls, 2);
        zassert_equal(queue.try_put_calls, 3);
        zassert_equal(queue.at(queue.size() - 1).data[0], 0x77);
    }
}

ZTEST(lcs_forward_queue, test_frame_content_unchanged)
{
    lcs::msg frame{};
    frame.board = 1;
    frame.data = {1, 2, 3, 4, 5, 6, 7, 8};
    FakeQueue<1> queue;

    lcs::put_purging(queue, frame);

    zassert_equal(queue.size(), 1);
    zassert_equal(queue.at(0).board, 1);
    for (std::size_t i = 0; i < 8; ++i)
        zassert_equal(queue.at(0).data[i], static_cast<uint8_t>(i + 1));
}
