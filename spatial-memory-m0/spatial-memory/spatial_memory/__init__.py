"""
spatial_memory — 空间记忆框架 M0

组装方式(依赖注入,所有部件可单独替换):

    from spatial_memory import build_system
    sys = build_system("memory.db")
    sys.consolidator.ingest(observation)      # 写路径
    sys.query.semantic_search("红色灭火器")    # 读路径
"""
from dataclasses import dataclass

from .consolidator import Consolidator, ConsolidatorConfig
from .query import SpatialQuery
from .schema import *  # noqa
from .store import (EntityStore, EventLog, LocalBlobStore, NumpyVectorIndex,
                    SqliteStore, VectorIndex)
from .synthetic import SimulatedRobot, SyntheticWorld, toy_embed


@dataclass
class SpatialMemorySystem:
    entities: EntityStore
    events: EventLog
    vindex: VectorIndex
    consolidator: Consolidator
    query: SpatialQuery


def build_system(db_path: str = ":memory:",
                 embed_fn=toy_embed,
                 config: ConsolidatorConfig | None = None) -> SpatialMemorySystem:
    store = SqliteStore(db_path)          # 同时充当 EntityStore + EventLog
    vindex = NumpyVectorIndex()
    consolidator = Consolidator(store, store, vindex, config)
    query = SpatialQuery(store, store, vindex, embed_fn=embed_fn)
    return SpatialMemorySystem(store, store, vindex, consolidator, query)
