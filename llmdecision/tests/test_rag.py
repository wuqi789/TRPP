from __future__ import annotations

import sys
import unittest
from pathlib import Path

MODULE_ROOT = Path(__file__).resolve().parents[1]
if str(MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(MODULE_ROOT))

from rag.knowledge_base import KnowledgeBase, KnowledgeCase
from rag.retriever import LocalJSONRetriever


def case(
    case_id: str,
    object_class: str,
    material: str,
    keywords: tuple[str, ...],
) -> KnowledgeCase:
    return KnowledgeCase(
        case_id=case_id,
        object_class=object_class,
        material=material,
        description=f"Historical {object_class} case",
        push_success=True,
        push_force_range=(10.0, 20.0),
        risks=("test_risk",),
        keywords=keywords,
    )


class RetrieverTests(unittest.TestCase):
    def setUp(self) -> None:
        self.query = "A cardboard box blocks the planned path."

    def test_returns_ranked_top_k_from_scene_text(self) -> None:
        retriever = LocalJSONRetriever(
            KnowledgeBase(
                [
                    case("cardboard", "cardboard_box", "cardboard", ("box",)),
                    case("plastic", "plastic_box", "plastic", ("box",)),
                    case("chair", "chair", "metal", ("chair",)),
                ]
            )
        )
        results = retriever.retrieve(self.query, top_k=2)
        self.assertEqual(
            [result.case.case_id for result in results],
            ["cardboard", "plastic"],
        )
        self.assertGreater(results[0].score, results[1].score)
        self.assertIn("cardboard", results[0].matched_terms)
        self.assertIn("box", results[0].matched_terms)

    def test_empty_knowledge_base_returns_empty_results(self) -> None:
        retriever = LocalJSONRetriever(KnowledgeBase([]))
        self.assertEqual(retriever.retrieve(self.query, top_k=3), [])

    def test_query_must_not_be_empty(self) -> None:
        retriever = LocalJSONRetriever(KnowledgeBase([]))
        with self.assertRaisesRegex(ValueError, "non-empty string"):
            retriever.retrieve(" ", top_k=3)

    def test_top_k_must_be_positive(self) -> None:
        retriever = LocalJSONRetriever(KnowledgeBase([]))
        with self.assertRaisesRegex(ValueError, "positive integer"):
            retriever.retrieve(self.query, top_k=0)


if __name__ == "__main__":
    unittest.main()
