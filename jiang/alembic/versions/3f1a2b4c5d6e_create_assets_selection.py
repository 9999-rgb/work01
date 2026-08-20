"""create assets and selection tables

Revision ID: 3f1a2b4c5d6e
Revises: 75036a3d0043
Create Date: 2026-08-18 00:00:00.000000

"""
from typing import Sequence, Union

from alembic import op
import sqlalchemy as sa


# revision identifiers, used by Alembic.
revision: str = '3f1a2b4c5d6e'
down_revision: Union[str, Sequence[str], None] = '75036a3d0043'
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    """Upgrade schema."""
    # 资产目录索引：与 ``app.assets.models.Asset`` 一致（目录 + 选择）。
    op.create_table(
        'assets',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('kind', sa.String(length=16), nullable=False),
        sa.Column('name', sa.String(length=64), nullable=False),
        sa.Column('version', sa.String(length=32), nullable=False),
        sa.Column('description', sa.String(length=256), nullable=False),
        sa.Column('path', sa.String(length=256), nullable=False),
        sa.Column('files', sa.JSON(), nullable=False),
        sa.Column('references', sa.JSON(), nullable=False),
        sa.Column('imported_at', sa.String(length=40), nullable=False),
        sa.Column('validated', sa.Boolean(), nullable=False),
        sa.PrimaryKeyConstraint('id'),
        sa.UniqueConstraint('kind', 'name', name='uq_assets_kind_name'),
    )
    op.create_index(op.f('ix_assets_kind'), 'assets', ['kind'], unique=False)

    # 单行选择表：与 ``app.assets.models.Selection`` 一致。
    op.create_table(
        'selection',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('scene', sa.String(length=64), nullable=True),
        sa.Column('cabinet', sa.String(length=64), nullable=True),
        sa.PrimaryKeyConstraint('id'),
    )


def downgrade() -> None:
    """Downgrade schema."""
    op.drop_table('selection')
    op.drop_index(op.f('ix_assets_kind'), table_name='assets')
    op.drop_table('assets')
