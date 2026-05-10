from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Optional

import yaml


@dataclass(frozen=True)
class Product:
    id: str
    name: str
    category: str
    price: float
    aliases: List[str] = field(default_factory=list)
    priority_for_intents: Dict[str, float] = field(default_factory=dict)
    selling_points: List[str] = field(default_factory=list)
    suitable_needs: List[str] = field(default_factory=list)


class ProductCatalog:
    def __init__(self, products: Iterable[Product]) -> None:
        self.products = list(products)
        self.by_id = {p.id: p for p in self.products}

    @classmethod
    def from_yaml(cls, path: str) -> 'ProductCatalog':
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        products = []
        for item in data.get('products', []):
            products.append(Product(
                id=str(item['id']),
                name=str(item['name']),
                category=str(item.get('category', '')),
                price=float(item['price']),
                aliases=[str(v) for v in item.get('aliases', [])],
                priority_for_intents={
                    str(k): float(v)
                    for k, v in (item.get('priority_for_intents', {}) or {}).items()
                },
                selling_points=[str(v) for v in item.get('selling_points', [])],
                suitable_needs=[str(v) for v in item.get('suitable_needs', [])],
            ))
        return cls(products)

    def names(self) -> List[str]:
        return [p.name for p in self.products]

    def match_text(self, text: str) -> Optional[Product]:
        normalized = text.lower().replace(' ', '')
        best = None
        best_len = 0
        for product in self.products:
            candidates = [product.name, product.id, product.category] + product.aliases
            for candidate in candidates:
                c = str(candidate).lower().replace(' ', '')
                if c and c in normalized and len(c) > best_len:
                    best = product
                    best_len = len(c)
        return best

    def get(self, product_id: str) -> Optional[Product]:
        return self.by_id.get(product_id)

    def score_for_need(self, product: Product, need: str, preferred: List[str]) -> float:
        score = product.priority_for_intents.get(need, 0.0)
        tokens = [product.name, product.category, product.id] + product.aliases
        for rank, item in enumerate(preferred):
            item_norm = str(item).lower().replace(' ', '')
            if not item_norm:
                continue
            for token in tokens:
                token_norm = str(token).lower().replace(' ', '')
                if item_norm in token_norm or token_norm in item_norm:
                    score += max(1.0, 100.0 - rank * 10.0)
                    break
        return score


def product_to_dict(product: Product) -> Dict[str, Any]:
    return {
        'item_id': product.id,
        'item_name': product.name,
        'category': product.category,
        'price': product.price,
        'aliases': list(product.aliases),
        'priority_for_intents': dict(product.priority_for_intents),
        'selling_points': list(product.selling_points),
        'suitable_needs': list(product.suitable_needs),
    }
