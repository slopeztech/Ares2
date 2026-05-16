/**
 * @file  main.cpp
 * @brief Unity runner for test_dispatcher tests.
 *
 * Test count: 5
 */
#include <unity.h>

void setUp()    {}
void tearDown() {}

// ── test_fragment_reassembly.cpp ─────────────────────────────────────────────

extern void test_frag_two_segments_reassemble();
extern void test_frag_orphaned_session_no_phantom_execution();
extern void test_frag_session_timeout_discards_and_restarts();
extern void test_frag_session_collision_sends_nack();
extern void test_frag_orphaned_session_expires_and_slot_recycled();

// ── Runner ───────────────────────────────────────────────────────────────────

int main()
{
    UNITY_BEGIN();

    // fragment reassembly
    RUN_TEST(test_frag_two_segments_reassemble);
    RUN_TEST(test_frag_orphaned_session_no_phantom_execution);
    RUN_TEST(test_frag_session_timeout_discards_and_restarts);
    RUN_TEST(test_frag_session_collision_sends_nack);
    RUN_TEST(test_frag_orphaned_session_expires_and_slot_recycled);

    return UNITY_END();
}
