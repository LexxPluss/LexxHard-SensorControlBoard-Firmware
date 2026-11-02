/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Overflow policy of the uplink forward queue. Pure: the queue is passed in.
 * A frame that finds the queue full empties the whole queue first, so the consumer always sees the newest
 * frames and the producer never blocks. The dropped frames are not counted.
 */

#pragma once

namespace lexxhard::lcs {

/* Queue needs `bool try_put(const Frame &)` (true when stored) and `void purge()` (drops every queued frame). */
template <typename Queue, typename Frame>
void put_purging(Queue &queue, const Frame &frame)
{
    while (!queue.try_put(frame))
        queue.purge();
}

}
