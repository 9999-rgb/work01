"""add toolset column to selection

Revision ID: 9c1d2e3f4a5b
Revises: 5d7e8f9a0b1c
Create Date: 2026-08-21 00:00:00.000000

"""
from typing import Sequence, Union

from alembic import op
import sqlalchemy as sa


# revision identifiers, used by Alembic.
revision: str = '9c1d2e3f4a5b'
down_revision: Union[str, Sequence[str], None] = '5d7e8f9a0b1c'
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    """Upgrade schema."""
    # 末端工具套装 A / B（None = 未选择，视为 A）。与
    # ``app.assets.models.Selection.toolset`` 一致。
    op.add_column(
        'selection',
        sa.Column('toolset', sa.String(length=2), nullable=True),
    )


def downgrade() -> None:
    """Downgrade schema."""
    op.drop_column('selection', 'toolset')
