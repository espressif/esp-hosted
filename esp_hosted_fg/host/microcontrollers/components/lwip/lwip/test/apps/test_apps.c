#include "lwip_check.h"

#include "api/test_sockets.h"

#include "lwip/init.h"
#include "lwip/tcpip.h"

Suite* create_suite(const char* name, testfunc *tests, size_t num_tests, SFun setup, SFun teardown)
{
  size_t i;
  Suite *s = suite_create(name);

  for(i = 0; i < num_tests; i++) {
    TCase *tc_core = tcase_create(name);
    if ((setup != NULL) || (teardown != NULL)) {
      tcase_add_checked_fixture(tc_core, setup, teardown);
    }
    tcase_add_named_test(tc_core, tests[i]);
    suite_add_tcase(s, tc_core);
  }
  return s;
}

void lwip_check_ensure_no_alloc(unsigned int skip)
{
  int i;
  unsigned int mask;
  for (i = 0, mask = 1; i < MEMP_MAX; i++, mask <<= 1) {
    if (!(skip & mask)) {
      fail_unless(lwip_stats.memp[i]->used == 0);
    }
  }
}

int main(void)
{
  int number_failed;
  SRunner *sr;
  size_t i;
  suite_getter_fn* suites[] = {
    sockets_suite,
  };
  size_t num = sizeof(suites)/sizeof(void*);
  LWIP_ASSERT("No suites defined", num > 0);

  tcpip_init(NULL, NULL);

  sr = srunner_create((suites[0])());
  srunner_set_xml(sr, "lwip_test_apps.xml");
  for(i = 1; i < num; i++) {
    srunner_add_suite(sr, ((suite_getter_fn*)suites[i])());
  }

  srunner_set_fork_status(sr, CK_NOFORK);

  srunner_run_all(sr, CK_NORMAL);
  number_failed = srunner_ntests_failed(sr);
  srunner_free(sr);
  return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
