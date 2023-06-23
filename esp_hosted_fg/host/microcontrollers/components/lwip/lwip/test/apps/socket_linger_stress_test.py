try:
    import sys
    from junit_xml import TestSuite as ts, TestCase as tc

    t=tc("lingering close stress test")
    if len(sys.argv) > 1 and sys.argv[1] == "failed":
        t.add_failure_info("test got stuck when closing clients socket")
    print(ts.to_xml_string([ts("SOCKET SO_LINGER stress test", [t])]))

except ImportError:
    print()
