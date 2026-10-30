"""実測CSVの単位・集計条件・欠測を誤って受理しないことを確認する。"""
import csv
import json
from pathlib import Path
import tempfile
import unittest
from prepare_receiver_curve import prepare, aggregate_samples


class ReceiverCurveTest(unittest.TestCase):
    def test_native_csv_and_rejections(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for i, per in enumerate((.01, .2)):
                (root/f'capture{i}.csv').write_text('time,packet,bER,PER,SNR,RSSI,MCS\n'
                    f'1,100,1e-5,{per},10,200,8\n')
            header = ['direction','frame','mcs','frame_bytes','attenuation_db',
                      'reference_attenuation_db','reference_rssi_dbm','statistics_csv','aggregation','ber_stage']
            conditions = [['P_B_TO_A','DATA',8,2048,i*5,0,-60,f'capture{i}.csv','averaged','unknown'] for i in range(2)]
            def save():
                with (root/'manifest.csv').open('w',newline='') as stream:
                    writer=csv.writer(stream);writer.writerow(header);writer.writerows(conditions)
            save()
            self.assertEqual(prepare(root/'manifest.csv', root/'curve.csv'), 2)
            with (root/'curve.csv').open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual([float(r['rssi_dbm']) for r in rows], [-65,-60])
            self.assertEqual([float(r['per']) for r in rows], [.2,.01])
            self.assertTrue((root/'curve.csv.provenance.json').exists())
            with self.assertRaisesRegex(ValueError,'既存'):
                prepare(root/'manifest.csv', root/'curve.csv')
            with self.assertRaisesRegex(ValueError,'測定位置'):
                prepare(root/'manifest.csv', root/'ber.csv', metric='ber')
            self.assertEqual(prepare(root/'manifest.csv', root/'ber.csv', metric='ber',allow_unknown_ber=True), 2)
            conditions[0][-2]='clear_write';save()
            with self.assertRaisesRegex(ValueError,'packet_lost'):
                prepare(root/'manifest.csv', root/'bad.csv')
            conditions[0][-2]='averaged';save()
            (root/'capture1.csv').write_text('time,packet,bER,PER,SNR,RSSI,MCS\n1,100,0,0,20,200,8\n2,nan,nan,nan,nan,nan,nan\n')
            with self.assertRaises(ValueError):
                prepare(root/'manifest.csv', root/'bad.csv')

    def test_clear_write_weighting_and_missing_lost(self):
        rows = [dict(time='1', packet='10', bER='.1', PER='.2', RSSI='100', MCS='8'),
                dict(time='2', packet='90', bER='.01', PER='.8', RSSI='101', MCS='8')]
        ber, info = aggregate_samples(rows, 8, 'ber', 'clear_write')
        self.assertAlmostEqual(ber, .019)
        self.assertEqual(info['denominator_sum'], 100)
        with self.assertRaisesRegex(ValueError, 'packet_lost'):
            aggregate_samples(rows, 8, 'per', 'clear_write')
        per, info = aggregate_samples(rows, 8, 'per', 'clear_write', True)
        self.assertAlmostEqual(per, .74)
        self.assertIn('approximation', info['method'])
        rows[0]['packet_lost'] = '10'
        rows[1]['packet_lost'] = '0'
        per, info = aggregate_samples(rows, 8, 'per', 'clear_write')
        self.assertAlmostEqual(per, 76 / 110)
        self.assertEqual(info['method'], 'detected_plus_lost_weighted_per')
        rows[0]['MCS'] = '6'
        with self.assertRaisesRegex(ValueError, 'MCS'):
            aggregate_samples(rows, 8, 'ber', 'clear_write')
        rows[0]['MCS'] = '8'
        rows[0]['packet'] = '0'
        with self.assertRaisesRegex(ValueError, '分母0'):
            aggregate_samples(rows, 8, 'ber', 'clear_write')
        rows[0]['packet'] = '10'
        rows[0]['bER'] = 'nan'
        with self.assertRaisesRegex(ValueError, '有限値'):
            aggregate_samples(rows, 8, 'ber', 'clear_write')
        rows[0]['bER'] = '.1'
        rows[1]['time'] = '1'
        with self.assertRaisesRegex(ValueError, '単調増加'):
            aggregate_samples(rows, 8, 'ber', 'clear_write')

    def test_relative_curve_keeps_unknown_dbm_out(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'capture.csv').write_text('time,packet,bER,PER,SNR,RSSI,MCS\n'
                                           '1,10,.1,.2,10,100,8\n2,90,.01,.8,10,101,8\n')
            with (root/'manifest.csv').open('w', newline='') as stream:
                writer = csv.writer(stream)
                writer.writerow(['direction','frame','mcs','frame_bytes','attenuation_db',
                                 'reference_attenuation_db','reference_rssi_dbm','statistics_csv',
                                 'aggregation','ber_stage'])
                for attenuation in (0, 2):
                    writer.writerow(['Q_A_TO_B','DATA',8,100000,attenuation,0,'',
                                     'capture.csv','clear_write','unknown'])
            self.assertEqual(prepare(root/'manifest.csv', root/'relative.csv',
                             allow_detect_weighted_per=True, relative_power=True), 2)
            with (root/'relative.csv').open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertNotIn('rssi_dbm', rows[0])
            self.assertEqual([float(r['relative_power_db']) for r in rows], [-2, 0])
            self.assertAlmostEqual(float(rows[0]['per']), .74)
            meta = json.loads((root/'relative.csv.provenance.json').read_text())
            self.assertFalse(meta['simulator_schema_compatible'])
            self.assertEqual(meta['sources'][0]['aggregation']['used_rows'], 2)
            with self.assertRaises(ValueError):
                prepare(root/'manifest.csv', root/'absolute.csv', allow_detect_weighted_per=True)


if __name__ == '__main__':
    unittest.main()
