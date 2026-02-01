import { useRouter } from "expo-router";
import { useCallback, useState } from "react";

export function useDebouncedNavigation() {
  const router = useRouter();
  const [isNavigating, setIsNavigating] = useState(false);

  const push = useCallback(
    (...args: Parameters<typeof router.push>) => {
      if (isNavigating) return;
      setIsNavigating(true);
      router.push(...args);
      setTimeout(() => {
        setIsNavigating(false);
      }, 1000);
    },
    [isNavigating, router]
  );

  const replace = useCallback(
    (...args: Parameters<typeof router.replace>) => {
      if (isNavigating) return;
      setIsNavigating(true);
      router.replace(...args);
      setTimeout(() => {
        setIsNavigating(false);
      }, 1000);
    },
    [isNavigating, router]
  );

  return { push, replace, isNavigating };
}
