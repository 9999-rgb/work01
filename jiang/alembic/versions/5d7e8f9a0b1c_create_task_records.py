"""create task_records and task_progress_events tables

Revision ID: 5d7e8f9a0b1c
Revises: 3f1a2b4c5d6e
Create Date: 2026-08-19 00:00:00.000000

"""
from typing import Sequence, Union

from alembic import op
import sqlalchemy as sa


# revision identifiers, used by Alembic.
revision: str = '5d7e8f9a0b1c'
down_revision: Union[str, Sequence[str], None] = '3f1a2b4c5d6e'
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    """Upgrade schema."""
    # 任务主表：每任务一行最终快照，与 ``app.tasks.models.TaskRecord`` 一致。
    op.create_table(
        'task_records',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('task_id', sa.String(length=40), nullable=False),
        sa.Column('type', sa.String(length=16), nullable=False),
        sa.Column('status', sa.String(length=16), nullable=False),
        sa.Column('phase', sa.String(length=32), nullable=False),
        sa.Column('progress', sa.Float(), nullable=False),
        sa.Column('message', sa.String(length=512), nullable=False),
        sa.Column('cabinet', sa.String(length=64), nullable=True),
        sa.Column('control_id', sa.String(length=64), nullable=True),
        sa.Column('control_name', sa.String(length=64), nullable=True),
        sa.Column('command', sa.String(length=16), nullable=True),
        sa.Column('target_state', sa.String(length=64), nullable=True),
        sa.Column('target_position', sa.Float(), nullable=True),
        sa.Column('force', sa.Float(), nullable=True),
        sa.Column('request', sa.JSON(), nullable=False),
        sa.Column('result', sa.JSON(), nullable=True),
        sa.Column('business_data', sa.JSON(), nullable=True),
        sa.Column('failure_code', sa.String(length=64), nullable=True),
        sa.Column('failure_reason', sa.String(length=512), nullable=True),
        sa.Column('failure_details', sa.JSON(), nullable=True),
        sa.Column('duration_seconds', sa.Float(), nullable=True),
        sa.Column('created_at', sa.Float(), nullable=False),
        sa.Column('started_at', sa.Float(), nullable=True),
        sa.Column('completed_at', sa.Float(), nullable=True),
        sa.Column('updated_at', sa.Float(), nullable=False),
        sa.PrimaryKeyConstraint('id'),
    )
    op.create_index(op.f('ix_task_records_task_id'), 'task_records', ['task_id'], unique=True)
    op.create_index(op.f('ix_task_records_type'), 'task_records', ['type'], unique=False)
    op.create_index(op.f('ix_task_records_status'), 'task_records', ['status'], unique=False)
    op.create_index(op.f('ix_task_records_cabinet'), 'task_records', ['cabinet'], unique=False)
    op.create_index(op.f('ix_task_records_control_id'), 'task_records', ['control_id'], unique=False)
    op.create_index(op.f('ix_task_records_failure_code'), 'task_records', ['failure_code'], unique=False)
    op.create_index(op.f('ix_task_records_created_at'), 'task_records', ['created_at'], unique=False)

    # 进度事件明细表：~1Hz 全量，按 id 自增排序还原时间线。
    op.create_table(
        'task_progress_events',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('task_id', sa.String(length=40), nullable=False),
        sa.Column('sequence', sa.Integer(), nullable=True),
        sa.Column('event', sa.String(length=32), nullable=False),
        sa.Column('status', sa.String(length=16), nullable=False),
        sa.Column('phase', sa.String(length=32), nullable=False),
        sa.Column('progress', sa.Float(), nullable=False),
        sa.Column('message', sa.String(length=512), nullable=False),
        sa.Column('business_data', sa.JSON(), nullable=True),
        sa.Column('timestamp', sa.Float(), nullable=False),
        sa.Column('elapsed_seconds', sa.Float(), nullable=True),
        sa.PrimaryKeyConstraint('id'),
    )
    op.create_index(op.f('ix_task_progress_events_task_id'), 'task_progress_events', ['task_id'], unique=False)
    op.create_index(op.f('ix_task_progress_events_timestamp'), 'task_progress_events', ['timestamp'], unique=False)


def downgrade() -> None:
    """Downgrade schema."""
    op.drop_index(op.f('ix_task_progress_events_timestamp'), table_name='task_progress_events')
    op.drop_index(op.f('ix_task_progress_events_task_id'), table_name='task_progress_events')
    op.drop_table('task_progress_events')
    op.drop_index(op.f('ix_task_records_created_at'), table_name='task_records')
    op.drop_index(op.f('ix_task_records_failure_code'), table_name='task_records')
    op.drop_index(op.f('ix_task_records_control_id'), table_name='task_records')
    op.drop_index(op.f('ix_task_records_cabinet'), table_name='task_records')
    op.drop_index(op.f('ix_task_records_status'), table_name='task_records')
    op.drop_index(op.f('ix_task_records_type'), table_name='task_records')
    op.drop_index(op.f('ix_task_records_task_id'), table_name='task_records')
    op.drop_table('task_records')
