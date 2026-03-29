import Colors from "@/constants/colors";
import { useAuth } from "@/contexts/AuthContext";
import { ArrowLeft, Check } from "lucide-react-native";
import React, { useState } from "react";
import {
	ActivityIndicator,
	Alert,
	StyleSheet,
	Text,
	TextInput,
	TouchableOpacity,
	View,
} from "react-native";
import { useSafeAreaInsets } from "react-native-safe-area-context";
import { router } from "expo-router";

export default function EditProfileScreen() {
	const insets = useSafeAreaInsets();
	const { user, updateUser } = useAuth();
	const [name, setName] = useState(user?.name ?? "");
	const [isSaving, setIsSaving] = useState(false);

	const hasChanges = name.trim() !== (user?.name ?? "");
	const isValid = name.trim().length >= 2;

	const handleSave = async () => {
		if (!isValid || !hasChanges) return;
		setIsSaving(true);
		try {
			await updateUser({ name: name.trim() });
			router.back();
		} catch {
			Alert.alert("Error", "Failed to update profile. Please try again.");
		} finally {
			setIsSaving(false);
		}
	};

	return (
		<View style={[styles.container, { paddingTop: insets.top }]}>
			<View style={styles.header}>
				<TouchableOpacity onPress={() => router.back()} style={styles.backBtn}>
					<ArrowLeft size={24} color={Colors.text} />
				</TouchableOpacity>
				<Text style={styles.headerTitle}>Edit Profile</Text>
				<TouchableOpacity
					onPress={handleSave}
					disabled={!hasChanges || !isValid || isSaving}
					style={[
						styles.saveBtn,
						(!hasChanges || !isValid) && styles.saveBtnDisabled,
					]}
				>
					{isSaving ? (
						<ActivityIndicator size="small" color={Colors.accent} />
					) : (
						<Check
							size={24}
							color={
								hasChanges && isValid ? Colors.accent : Colors.textSecondary
							}
						/>
					)}
				</TouchableOpacity>
			</View>

			<View style={styles.content}>
				<View style={styles.avatarSection}>
					<View style={styles.avatar}>
						<Text style={styles.avatarText}>
							{name.trim() ? name.trim()[0].toUpperCase() : "?"}
						</Text>
					</View>
				</View>

				<View style={styles.fieldGroup}>
					<Text style={styles.fieldLabel}>DISPLAY NAME</Text>
					<View style={styles.inputContainer}>
						<TextInput
							style={styles.input}
							value={name}
							onChangeText={setName}
							placeholder="Enter your name"
							placeholderTextColor={Colors.textSecondary}
							autoFocus
							returnKeyType="done"
							onSubmitEditing={handleSave}
							maxLength={50}
						/>
					</View>
					<Text style={styles.fieldHint}>
						This is purely cosmetic and for logbooking
					</Text>
				</View>

				<View style={styles.fieldGroup}>
					<Text style={styles.fieldLabel}>EMAIL</Text>
					<View style={[styles.inputContainer, styles.inputDisabled]}>
						<Text style={styles.inputDisabledText}>
							{user?.email ?? "Not set"}
						</Text>
					</View>
					<Text style={styles.fieldHint}>
						Managed by Apple Sign-In. Cannot be changed here.
					</Text>
				</View>
			</View>
		</View>
	);
}

const styles = StyleSheet.create({
	container: {
		flex: 1,
		backgroundColor: Colors.background,
	},
	header: {
		flexDirection: "row",
		alignItems: "center",
		justifyContent: "space-between",
		paddingHorizontal: 20,
		paddingVertical: 16,
		borderBottomWidth: 1,
		borderBottomColor: Colors.border,
	},
	backBtn: {
		width: 40,
		height: 40,
		alignItems: "center",
		justifyContent: "center",
	},
	headerTitle: {
		fontSize: 18,
		fontWeight: "700",
		color: Colors.text,
	},
	saveBtn: {
		width: 40,
		height: 40,
		alignItems: "center",
		justifyContent: "center",
	},
	saveBtnDisabled: {
		opacity: 0.5,
	},
	content: {
		padding: 20,
	},
	avatarSection: {
		alignItems: "center",
		marginBottom: 32,
		marginTop: 12,
	},
	avatar: {
		width: 88,
		height: 88,
		borderRadius: 44,
		backgroundColor: Colors.accent,
		alignItems: "center",
		justifyContent: "center",
	},
	avatarText: {
		fontSize: 36,
		fontWeight: "700",
		color: Colors.black,
	},
	fieldGroup: {
		marginBottom: 24,
	},
	fieldLabel: {
		fontSize: 12,
		fontWeight: "600",
		color: Colors.textSecondary,
		letterSpacing: 0.8,
		marginBottom: 8,
		paddingLeft: 4,
	},
	inputContainer: {
		backgroundColor: Colors.surface,
		borderRadius: 14,
		borderWidth: 1,
		borderColor: Colors.border,
		paddingHorizontal: 16,
		paddingVertical: 14,
	},
	input: {
		fontSize: 16,
		fontWeight: "500",
		color: Colors.text,
	},
	inputDisabled: {
		opacity: 0.5,
	},
	inputDisabledText: {
		fontSize: 16,
		fontWeight: "500",
		color: Colors.textSecondary,
	},
	fieldHint: {
		fontSize: 12,
		color: Colors.textSecondary,
		marginTop: 8,
		paddingLeft: 4,
		lineHeight: 18,
	},
});
