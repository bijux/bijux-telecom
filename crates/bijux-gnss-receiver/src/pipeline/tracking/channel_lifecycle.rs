#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChannelState {
    Idle,
    Acquired,
    PullIn,
    Tracking,
    Degraded,
    Lost,
}

impl std::fmt::Display for ChannelState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let value = match self {
            Self::Idle => "idle",
            Self::Acquired => "acquired",
            Self::PullIn => "pull_in",
            Self::Tracking => "tracking",
            Self::Degraded => "degraded",
            Self::Lost => "lost",
        };
        write!(f, "{value}")
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TrackingChannelState {
    Acquired,
    PullIn,
    Locked,
    Degraded,
    Lost,
    Reacquired,
    Refused,
}

impl std::fmt::Display for TrackingChannelState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let value = match self {
            Self::Acquired => "acquired",
            Self::PullIn => "pull_in",
            Self::Locked => "locked",
            Self::Degraded => "degraded",
            Self::Lost => "lost",
            Self::Reacquired => "reacquired",
            Self::Refused => "refused",
        };
        write!(f, "{value}")
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TrackingChannelStateEvent {
    pub state: TrackingChannelState,
    pub epoch_idx: u64,
    pub sample_index: u64,
    pub reason: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TrackingChannelStateReport {
    pub sat: SatId,
    pub channel_id: u8,
    pub channel_uid: String,
    pub final_state: TrackingChannelState,
    pub final_reason: Option<String>,
    pub emitted_states: Vec<TrackingChannelStateEvent>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChannelEvent {
    Acquire,
    PullIn,
    Track,
    Lose,
    Reset,
}

#[derive(Debug, Clone)]
pub struct Channel {
    pub id: u8,
    pub state: ChannelState,
}

impl Channel {
    pub fn new(id: u8) -> Self {
        Self { id, state: ChannelState::Idle }
    }

    pub fn apply(&mut self, event: ChannelEvent) {
        let next = match (self.state, event) {
            (ChannelState::Idle, ChannelEvent::Acquire) => ChannelState::Acquired,
            (ChannelState::Acquired, ChannelEvent::PullIn) => ChannelState::PullIn,
            (ChannelState::PullIn, ChannelEvent::Track) => ChannelState::Tracking,
            (_, ChannelEvent::Lose) => ChannelState::Lost,
            (_, ChannelEvent::Reset) => ChannelState::Idle,
            (state, _) => state,
        };
        self.state = Tracking::transition_state(self.id, self.state, next);
    }
}
