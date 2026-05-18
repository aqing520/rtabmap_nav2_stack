from __future__ import annotations

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from bt468_rtk_driver.nmea import fix_quality_to_label, parse_nmea_line


class NMEATestCase(unittest.TestCase):
    def test_parse_gga(self) -> None:
        parsed = parse_nmea_line(
            "$GNGGA,090020.00,2240.8406477,N,11402.7075007,E,1,12,0.48,75.358,M,-2.521,M,,*6D"
        )
        self.assertIsNotNone(parsed)
        assert parsed is not None
        self.assertEqual(parsed.type, "GGA")
        self.assertEqual(parsed.talker, "GN")
        self.assertEqual(parsed.fields["fix_quality"], 1)
        self.assertEqual(parsed.fields["fix_status"], "普通定位")
        self.assertEqual(parsed.fields["num_sv"], 12)
        self.assertAlmostEqual(parsed.fields["lat"], 22.680677461666668)
        self.assertAlmostEqual(parsed.fields["lon"], 114.04512501166667)

    def test_parse_rmc(self) -> None:
        parsed = parse_nmea_line(
            "$GNRMC,090020.00,A,2240.8406477,N,11402.7075007,E,0.005,,140520,,,A,V*16"
        )
        self.assertIsNotNone(parsed)
        assert parsed is not None
        self.assertEqual(parsed.type, "RMC")
        self.assertEqual(parsed.fields["status"], "A")
        self.assertEqual(parsed.fields["speed_knots"], 0.005)

    def test_parse_vtg(self) -> None:
        parsed = parse_nmea_line("$GNVTG,,T,,M,0.005,N,0.009,K,A*31")
        self.assertIsNotNone(parsed)
        assert parsed is not None
        self.assertEqual(parsed.type, "VTG")
        self.assertEqual(parsed.fields["speed_knots"], 0.005)
        self.assertEqual(parsed.fields["speed_kmh"], 0.009)

    def test_invalid_checksum_is_reported(self) -> None:
        parsed = parse_nmea_line("$GNVTG,,T,,M,0.005,N,0.009,K,A*00")
        self.assertIsNotNone(parsed)
        assert parsed is not None
        self.assertFalse(parsed.fields["checksum_valid"])

    def test_fix_quality_labels(self) -> None:
        self.assertEqual(fix_quality_to_label(0), "没定位")
        self.assertEqual(fix_quality_to_label(1), "普通定位")
        self.assertEqual(fix_quality_to_label(2), "DGPS")
        self.assertEqual(fix_quality_to_label(5), "RTK Float")
        self.assertEqual(fix_quality_to_label(4), "RTK Fixed")


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
