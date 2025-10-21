import { Ride } from "../types/ride";

export const MOCK_PAST_RIDES: Ride[] = [
  {
    id: "1",
    pickupLocationId: "4",
    dropoffLocationId: "1",
    status: "completed",
    requestedAt: new Date(Date.now() - 2 * 24 * 60 * 60 * 1000),
    pickupAt: new Date(Date.now() - 2 * 24 * 60 * 60 * 1000 + 5 * 60 * 1000),
    completedAt: new Date(
      Date.now() - 2 * 24 * 60 * 60 * 1000 + 12 * 60 * 1000
    ),
    vehicleId: "ACT-001",
  },
  {
    id: "2",
    pickupLocationId: "2",
    dropoffLocationId: "6",
    status: "completed",
    requestedAt: new Date(Date.now() - 3 * 24 * 60 * 60 * 1000),
    pickupAt: new Date(Date.now() - 3 * 24 * 60 * 60 * 1000 + 3 * 60 * 1000),
    completedAt: new Date(
      Date.now() - 3 * 24 * 60 * 60 * 1000 + 10 * 60 * 1000
    ),
    vehicleId: "ACT-002",
  },
  {
    id: "3",
    pickupLocationId: "7",
    dropoffLocationId: "5",
    status: "completed",
    requestedAt: new Date(Date.now() - 5 * 24 * 60 * 60 * 1000),
    pickupAt: new Date(Date.now() - 5 * 24 * 60 * 60 * 1000 + 4 * 60 * 1000),
    completedAt: new Date(
      Date.now() - 5 * 24 * 60 * 60 * 1000 + 15 * 60 * 1000
    ),
    vehicleId: "ACT-001",
  },
  {
    id: "4",
    pickupLocationId: "3",
    dropoffLocationId: "2",
    status: "completed",
    requestedAt: new Date(Date.now() - 7 * 24 * 60 * 60 * 1000),
    pickupAt: new Date(Date.now() - 7 * 24 * 60 * 60 * 1000 + 6 * 60 * 1000),
    completedAt: new Date(
      Date.now() - 7 * 24 * 60 * 60 * 1000 + 14 * 60 * 1000
    ),
    vehicleId: "ACT-003",
  },
];
