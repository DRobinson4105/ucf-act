export type RideStatus =
  | "pending"
  | "assigned"
  | "arriving"
  | "in-progress"
  | "completed"
  | "cancelled";

export interface Ride {
  id: string;
  pickupLocationId: string;
  dropoffLocationId: string;
  status: RideStatus;
  requestedAt: Date;
  pickupAt?: Date;
  completedAt?: Date;
  vehicleId?: string;
  estimatedWaitTime?: number;
  vehiclePosition?: { latitude: number; longitude: number };
}

export interface Notification {
  id: string;
  title: string;
  message: string;
  timestamp: Date;
  read: boolean;
  type: "ride-update" | "system" | "promotion";
}
