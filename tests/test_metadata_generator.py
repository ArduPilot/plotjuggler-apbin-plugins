"""Check offline JSON header generation and portable raw-string boundaries."""
import importlib.util
import json
import lzma
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

SCRIPT = (Path(__file__).resolve().parents[1] / 'DataLoadAPBin/scripts' /
          'generate_LogMessageDescriptions_header.py')
spec = importlib.util.spec_from_file_location('log_descriptions', SCRIPT)
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)


class MetadataGeneratorTest(unittest.TestCase):
    def test_utf8_chunk_boundary(self):
        text = 'a' * 15999 + 'ü' + 'b' * 17000
        chunks = generator.split_utf8_chunks(text)
        self.assertEqual(''.join(chunks), text)
        self.assertTrue(all(len(chunk.encode('utf-8')) <= 16000 for chunk in chunks))

    def test_generates_valid_lf_header_and_rejects_invalid_snapshot(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            content = json.dumps({'logformats': [{'name': 'TEST', 'description': 'µ )APJSON0',
                                                 'fields': [{'name': 'F', 'enum': {'name': 'Status',
                                                            'entries': [{'name': 'GOOD', 'value': 1,
                                                                         'description': 'Valid'}]}}]}]},
                                 ensure_ascii=False)
            for vehicle in ('Copter', 'Plane'):
                with lzma.open(root / f'{vehicle}_LogMessages.json.xz', 'wt', encoding='utf-8') as output:
                    output.write(content)
                (root / f'{vehicle}_LegacyDescriptions.json').write_text(content, encoding='utf-8')
            header = root / 'generated.h'
            subprocess.run([sys.executable, str(SCRIPT), str(root), str(header)], check=True)
            generated = header.read_bytes()
            self.assertNotIn(b'\r\n', generated)
            self.assertIn(b'R"APJSON1(', generated)
            self.assertIn('µ'.encode('utf-8'), generated)
            self.assertIn(b'"name":"GOOD"', generated)
            self.assertIn(b'"description":"Valid"', generated)

            with lzma.open(root / 'Copter_LogMessages.json.xz', 'wt', encoding='utf-8') as output:
                output.write('{}')
            result = subprocess.run([sys.executable, str(SCRIPT), str(root), str(header)],
                                    capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('expected a logformats array', result.stderr)


if __name__ == '__main__':
    unittest.main()
