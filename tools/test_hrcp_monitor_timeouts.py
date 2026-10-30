import tempfile
from pathlib import Path
import unittest
from hrcp_monitor_timeouts import TimeoutEvents, TimeoutHighlight


class TimeoutTest(unittest.TestCase):
    def test_incremental_exact_events_and_cache(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)
            f=p/'events.csv'
            f.write_text('time_s,event,frame,direction,reason\n1,TIMEOUT,REQUEST,A_TO_B,A_ATP_EXPIRED\n'
                         '2,ATP_DISCONNECT,,B_TO_A,B_ATP_EXPIRED\n3,TIMEOUT,ACK,A_TO_B,partial')
            events=TimeoutEvents(p);events.refresh()
            self.assertEqual([r['time_s'] for r in events.rows],[1])
            with f.open('a') as stream:stream.write('\n')
            (p/'completed').touch();events.refresh()
            self.assertEqual([r['time_s'] for r in events.rows],[1,3])
            self.assertEqual(TimeoutEvents(p).rows,events.rows)

    def test_highlight_crossing_pause_and_seek(self):
        h=TimeoutHighlight(2)
        self.assertIsNone(h.update([1,2],0,10,True))
        self.assertEqual(h.update([1,2],1.5,11,True),0)
        self.assertEqual(h.update([1,2],1.5,12,False),0)
        self.assertIsNone(h.update([1,2],1.5,13.1,False))
        self.assertIsNone(h.update([1,2],.5,14,False))
        self.assertEqual(h.update([1,2],2,15,False),1)
        self.assertIsNone(h.update([1,2],4,16,False))


if __name__=='__main__':unittest.main()
