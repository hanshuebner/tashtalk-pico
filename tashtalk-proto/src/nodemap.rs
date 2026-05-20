//! 256-bit node-ID bitmap.
//!
//! Wire layout (per `tashtalk/documentation/protocol.md`, `0x02 Set Node IDs`):
//! the 32 bytes following the opcode address node IDs 0..255, **LSB-first
//! within each byte**:
//!
//! - byte 0, bit 0 → node 0
//! - byte 0, bit 7 → node 7
//! - byte 1, bit 0 → node 8
//! - …
//! - byte 31, bit 7 → node 255
//!
//! Node IDs `0x00` and `0xFF` are not valid AppleTalk node IDs on
//! LocalTalk; the bit for `0xFF` *must* stay clear or broadcast frames
//! cannot be transmitted.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NodeBitmap {
    bytes: [u8; 32],
}

impl Default for NodeBitmap {
    fn default() -> Self {
        Self::empty()
    }
}

impl NodeBitmap {
    pub const fn empty() -> Self {
        Self { bytes: [0; 32] }
    }

    pub const fn from_bytes(bytes: [u8; 32]) -> Self {
        Self { bytes }
    }

    pub fn as_bytes(&self) -> &[u8; 32] {
        &self.bytes
    }

    pub fn set(&mut self, node_id: u8) {
        let (byte, bit) = idx(node_id);
        self.bytes[byte] |= 1 << bit;
    }

    pub fn clear(&mut self, node_id: u8) {
        let (byte, bit) = idx(node_id);
        self.bytes[byte] &= !(1 << bit);
    }

    pub fn contains(&self, node_id: u8) -> bool {
        let (byte, bit) = idx(node_id);
        self.bytes[byte] & (1 << bit) != 0
    }

    /// Iterate over the node IDs currently set in the bitmap.
    pub fn iter(&self) -> impl Iterator<Item = u8> + '_ {
        (0u16..=255).filter_map(move |n| {
            let n8 = n as u8;
            if self.contains(n8) {
                Some(n8)
            } else {
                None
            }
        })
    }
}

const fn idx(node_id: u8) -> (usize, u8) {
    ((node_id >> 3) as usize, node_id & 0x07)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_contains_nothing() {
        let bm = NodeBitmap::empty();
        for n in 0..=255u8 {
            assert!(!bm.contains(n));
        }
    }

    #[test]
    fn set_then_contains() {
        let mut bm = NodeBitmap::empty();
        bm.set(0);
        bm.set(7);
        bm.set(8);
        bm.set(255);
        assert!(bm.contains(0));
        assert!(bm.contains(7));
        assert!(bm.contains(8));
        assert!(bm.contains(255));
        assert!(!bm.contains(1));
        assert!(!bm.contains(254));
    }

    /// Verify wire layout matches the spec exactly. Setting node 0 must put
    /// the bit at byte 0, mask 0x01; setting node 7 → byte 0, mask 0x80;
    /// setting node 255 → byte 31, mask 0x80.
    #[test]
    fn wire_layout() {
        let mut bm = NodeBitmap::empty();
        bm.set(0);
        assert_eq!(bm.as_bytes()[0], 0x01);

        let mut bm = NodeBitmap::empty();
        bm.set(7);
        assert_eq!(bm.as_bytes()[0], 0x80);

        let mut bm = NodeBitmap::empty();
        bm.set(255);
        assert_eq!(bm.as_bytes()[31], 0x80);
    }

    #[test]
    fn iter_yields_set_ids() {
        let mut bm = NodeBitmap::empty();
        for id in [42, 100, 200] {
            bm.set(id);
        }
        let collected: heapless::Vec<u8, 8> = bm.iter().collect();
        assert_eq!(&collected[..], &[42, 100, 200]);
    }
}
